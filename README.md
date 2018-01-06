# krida_esp32
Esp32 arduino code to drive krida 8 port triac phase angle control board

Krida makes a nice 8 port triac board with arduino side sync pulses and good isolation.  They also seem to have used good snubber-less triacs allowing for negligible leakage current when the triacs are off.  I looked around but could not find any suitable esp family examples for driving this, and the esp32's ledc peripheral looked like it could naturally drive all 8 ports independently with ease.

pin mapping from esp32 to "Krida 8 Channel AC Light Dimmer Module Arduino"

```
GND -> ground of low voltage side
VCC -> device power, DC 5 volts
12 SYNC -> zero-cross detector output positive impulse (pulse length 200us)
14 CH1 -> triac gate input pin for 1 channel
27 CH2 -> triac gate input pin for 2 channel
26 CH3 -> triac gate input pin for 3 channel
25 CH4 -> triac gate input pin for 4 channel
33 CH5 -> triac gate input pin for 5 channel
32 CH6 -> triac gate input pin for 6 channel
16 CH7 -> triac gate input pin for 7 channel
17 CH8 -> triac gate input pin for 8 channel

AC INPUT -> AC voltage input 110/220
AC LOAD -> Connect your load here for 1ch, 2ch, 3ch, 4ch, 5ch, 6ch, 7ch and 8ch 
```
