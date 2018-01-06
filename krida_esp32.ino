/*
  Based on the ledc arduino example and shamelessly lifting from the esp32 arduino source,
  this version synchronises to the grid so that we can trigger triacs correctly for phase
  angle control.
  I initially used 35, 34 for outputs 6 and 7, but for some reason these
  don't seem to be able to connect to the ledc module.

  Connect pin 12 to the sync pin on the krida.
  I'm driving the krida with 5V and the trigger pins directly from the 3.3 port pins, no
  problems so far.

  It would be nice to be able to use the high level drivers directly, but I don't have the
  patience to push through the required changes right now.

  Nathan Hurst
*/

#include "esp32-hal.h"
#include "soc/dport_reg.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"
#include "driver/ledc.h"
#include "freertos/semphr.h"

uint8_t sync_pin = 12;
uint8_t triac_pins[] = {14, 27, 26, 25, 33, 32, 16, 17};
const int ledc_channel_base = 0;

#if CONFIG_DISABLE_HAL_LOCKS
#define LEDC_MUTEX_LOCK()
#define LEDC_MUTEX_UNLOCK()
#else
#define LEDC_MUTEX_LOCK()    do {} while (xSemaphoreTake(_ledc_sys_lock, portMAX_DELAY) != pdPASS)
#define LEDC_MUTEX_UNLOCK()  xSemaphoreGive(_ledc_sys_lock)
xSemaphoreHandle _ledc_sys_lock;
#endif

#define LEDC_CHAN(g,c) LEDC.channel_group[(g)].channel[(c)]
#define LEDC_TIMER(g,t) LEDC.timer_group[(g)].timer[(t)]

// modified from arduino esp32 code
static void _ledcSetupTimer(uint8_t chan, uint32_t div_num, uint8_t bit_num, bool apb_clk)
{
  uint8_t group = (chan / 8), timer = ((chan / 2) % 4);
  static bool tHasStarted = false;
  if (!tHasStarted) {
    tHasStarted = true;
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_LEDC_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);
    LEDC.conf.apb_clk_sel = 1;//LS use apb clock
#if !CONFIG_DISABLE_HAL_LOCKS
    _ledc_sys_lock = xSemaphoreCreateMutex();
#endif
  }
  LEDC_MUTEX_LOCK();
  LEDC_TIMER(group, timer).conf.div_num = div_num;//18 bit (10.8) This register is used to configure parameter for divider in timer the least significant eight bits represent the decimal part.
  LEDC_TIMER(group, timer).conf.bit_num = bit_num;//5 bit This register controls the range of the counter in timer. the counter range is [0 2**bit_num] the max bit width for counter is 20.
  LEDC_TIMER(group, timer).conf.tick_sel = apb_clk;//apb clock
  if (group) {
    LEDC_TIMER(group, timer).conf.low_speed_update = 1;//This bit is only useful for low speed timer channels, reserved for high speed timers
  }
  LEDC_TIMER(group, timer).conf.pause = 0;
  LEDC_TIMER(group, timer).conf.rst = 1;//This bit is used to reset timer the counter will be 0 after reset.
  LEDC_TIMER(group, timer).conf.rst = 0;
  LEDC_MUTEX_UNLOCK();
}

// modified from arduino esp32 code
static double _ledcSetupTimerFreq(uint8_t chan, double freq, uint8_t bit_num)
{
  uint64_t clk_freq = APB_CLK_FREQ;
  clk_freq <<= 8;//div_num is 8 bit decimal
  uint32_t div_num = (clk_freq >> bit_num) / freq;
  bool apb_clk = true;
  if (div_num > LEDC_DIV_NUM_HSTIMER0_V) {
    clk_freq /= 80;
    div_num = (clk_freq >> bit_num) / freq;
    if (div_num > LEDC_DIV_NUM_HSTIMER0_V) {
      div_num = LEDC_DIV_NUM_HSTIMER0_V;//lowest clock possible
    }
    apb_clk = false;
  } else if (div_num < 256) {
    div_num = 256;//highest clock possible
  }
  _ledcSetupTimer(chan, div_num, bit_num, apb_clk);
  //log_i("Fin: %f, Fclk: %uMhz, bits: %u, DIV: %u, Fout: %f",
  //        freq, apb_clk?80:1, bit_num, div_num, (clk_freq >> bit_num) / (double)div_num);
  return (clk_freq >> bit_num) / (double)div_num;
}

static void _ledcSetupChannel(uint8_t chan, uint8_t idle_level, uint8_t timer)
{
  uint8_t group = (chan / 8), channel = (chan % 8);
  LEDC_MUTEX_LOCK();
  LEDC_CHAN(group, channel).conf0.timer_sel = timer;//2 bit Selects the timer to attach 0-3
  LEDC_CHAN(group, channel).conf0.idle_lv = idle_level;//1 bit This bit is used to control the output value when channel is off.
  LEDC_CHAN(group, channel).hpoint.hpoint = 0;//20 bit The output value changes to high when timer selected by channel has reached hpoint
  LEDC_CHAN(group, channel).conf1.duty_inc = 1;//1 bit This register is used to increase the duty of output signal or decrease the duty of output signal for high speed channel
  LEDC_CHAN(group, channel).conf1.duty_num = 1;//10 bit This register is used to control the number of increased or decreased times for channel
  LEDC_CHAN(group, channel).conf1.duty_cycle = 1;//10 bit This register is used to increase or decrease the duty every duty_cycle cycles for channel
  LEDC_CHAN(group, channel).conf1.duty_scale = 0;//10 bit This register controls the increase or decrease step scale for channel.
  LEDC_CHAN(group, channel).duty.duty = 0;
  LEDC_CHAN(group, channel).conf0.sig_out_en = 0;//This is the output enable control bit for channel
  LEDC_CHAN(group, channel).conf1.duty_start = 0;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
  if (group) {
    LEDC_CHAN(group, channel).conf0.val &= ~BIT(4);
  } else {
    LEDC_CHAN(group, channel).conf0.clk_en = 0;
  }
  LEDC_MUTEX_UNLOCK();
}


// the setup routine runs once when you press reset:
void setup()
{
  Serial.begin(115200);
  delay(10);

  for (int i = 0; i < 8; i++) {
    int pin = triac_pins[i];
    int chan = ledc_channel_base + i;
    // Initialize channels
    // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
    // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
    _ledcSetupTimerFreq(chan, 120, 10); // 120 Hz PWM, 10-bit resolution
    _ledcSetupChannel(chan, LOW, 0);
    pinMode(pin, OUTPUT);
    pinMatrixOutAttach(pin, ((chan / 8) ? LEDC_LS_SIG_OUT0_IDX : LEDC_HS_SIG_OUT0_IDX) + (chan % 8),
                       true, false);
    //ledcAttachPin(triac_pins[i], ledc_channel+i); // assign RGB led pins to channels
    //gpio_matrix_out(triac_pins[i], LEDC_HS_SIG_OUT0_IDX + ledc_channel+i, 1, 0);
  }
  attachInterrupt(sync_pin, sync_pin_interrupt, RISING);
}

// catch the power wave
void sync_pin_interrupt() {
  // It's possible that all of these config bits could be set at the same time, 
  // perhaps the compiler works this out for me.
  for (int i = 0; i < 8; i++) {
    LEDC_TIMER(0, i).conf.rst = 1;
    LEDC_TIMER(0, i).conf.rst = 0;
  }
  // initial version used these, but the mapped structs seem to work well.
  //  SET_PERI_REG_MASK(LEDC_HSTIMER0_CONF_REG, LEDC_HSTIMER0_RST_M);
  //  CLEAR_PERI_REG_MASK(LEDC_HSTIMER0_CONF_REG, LEDC_HSTIMER0_RST_M);
}

int ramp(int x) {
  x = x % (3 * 1024);
  if (x < 1024) {
    return x;
  } else if (x < 2048) {
    return 2048 - x;
  }
  return 0;
}

void loop()
{
  // simple cross fading.
  for (int i = 0; i < 1024 * 3; i += 5) {
    for (int j = 0; j < 8; j++) {
      ledcWrite(j, ramp(i + 1024 * j));
    }

    delay(10); // We need to wait for the next cycle, around 8ms at 120Hz
  }
}
