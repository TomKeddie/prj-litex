# LEDD_ADR[3:0] Name      Usage                                     Access
#   1000        LEDDCR0   LED Driver Control Register 0               W
#   1001        LEDDBR    LED Driver Pre-scale Register               W
#   1010        LEDDONR   LED Driver ON Time Register                 W
#   1011        LEDDOFR   LED Driver OFF Time Register                W
#   0101        LEDDBCRR  LED Driver Breathe On Control Register      W
#   0110        LEDDBCFR  LED Driver Breathe Off Control Register     W
#   0001        LEDDPWRR  LED Driver Pulse Width Register for RED     W
#   0010        LEDDPWRG  LED Driver Pulse Width Register for GREEN   W
#   0011        LEDDPWRB  LED Driver Pulse Width Register for BLUE    W

LED Driver Control Register 0 (LEDDCR0)
LEDDCR0 can be written through LED Control Bus. 

Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
LEDDEN FR250 OUTPOL OUTSKEW QUICK STOP PWM MODE BRMSBEXT

Bit Field Description
7 LEDDEN LED Driver Enable Bit — This bit enables the LED Driver. If LEDDEN is cleared, The LED
Driver is disabled and the system clock into the LED Driver block will be gated off.
0 = LED Driver disabled
1 = LED Driver enabled
6 FR250 Flick Rate Select Bit — This bit selects the flick rate for the PWM logic between 125 Hz and
250Hz
0 = 125Hz
1 = 250Hz
5 OUTPOL PWM Outputs Polarity Select Bit — This bit selects the PWM outputs polarity.
0 = Active High
1 = Active Low
4 OUTSKEW PWM Output Skew Enable Bit — This bit enables the PWM slew to reduce simultaneous
switching noise, based on BRMSBEXT [1:0]
0 = Disable Output Skew
1 = Enable Output Skew:
3 QUICK STOP Blinking Sequence Quick Stop Enable Bit — This bit Enables the quick stop when
LEDD_EXE going low, instead of waiting current ON period finished when breathe on is enabled.
0 = Stop the blinking sequence when current ON period finished when LEDD_EXE goes low.
1 = Immediately terminate the blinking sequence after LEDD_EXE goes low. (within 5 ledd_clk
cycles)
2 PWM MODE 1 PWM Mode Selection Bit — This bit allow user to selection PWM mode between linear counter
approach, which results ‘square’ PWM pulse per flick cycle, or LFSR approach which results
PSUDO random PWM pulse per flick cycle (Spread Spectrum).
0 = Linear counter approach, which results ‘square’ PWM pulse per flick cycle
1 = LFSR approach which results PSUDO random PWM pulse per flick cycle (Spread Spectrum).
1:0 BRMSBEXT These two bits will serve as MSB of the Pre-scale Register to extend functional system clock frequency range. 
