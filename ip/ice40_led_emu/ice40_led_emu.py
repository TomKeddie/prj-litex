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

# =====================================================================
# LED Driver Control Register 0 (LEDDCR0)
# LEDDCR0 can be written through LED Control Bus. 
# Bit7   Bit6  Bit5   Bit4    Bit3       Bit2    Bit1/Bit0
# LEDDEN FR250 OUTPOL OUTSKEW QUICKSTOP  PWMMODE BRMSBEXT
# 
# Bit Field Description
#
# 7 LEDDEN LED Driver Enable Bit — This bit enables the LED Driver. If
# LEDDEN is cleared, The LED Driver is disabled and the system clock
# into the LED Driver block will be gated off.
#  0 = LED Driver disabled
#  1 = LED Driver enabled
#
# 6 FR250 Flick Rate Select Bit — This bit selects the flick rate for
# the PWM logic between 125 Hz and 250Hz
#  0 = 125Hz
#  1 = 250Hz
#
# 5 OUTPOL PWM Outputs Polarity Select Bit — This bit selects the PWM
# outputs polarity.
#  0 = Active High
#  1 = Active Low
#
# 4 OUTSKEW PWM Output Skew Enable Bit — This bit enables the PWM slew
# to reduce simultaneous switching noise, based on BRMSBEXT [1:0]
#  0 = Disable Output Skew
#  1 = Enable Output Skew:
#
# 3 QUICK STOP Blinking Sequence Quick Stop Enable Bit — This bit
# Enables the quick stop when LEDD_EXE going low, instead of waiting
# current ON period finished when breathe on is enabled.
#  0 = Stop the blinking sequence when current ON period finished when LEDD_EXE goes low.
#  1 = Immediately terminate the blinking sequence after LEDD_EXE goes low. (within 5 ledd_clk cycles)
#
# 2 PWM MODE 1 PWM Mode Selection Bit — This bit allow user to
# selection PWM mode between linear counter approach, which results
# ‘square’ PWM pulse per flick cycle, or LFSR approach which results
# PSUDO random PWM pulse per flick cycle (Spread Spectrum).
#  0 = Linear counter approach, which results ‘square’ PWM pulse per flick cycle
#  1 = LFSR approach which results PSUDO random PWM pulse per flick cycle (Spread Spectrum).
#
# 1:0 BRMSBEXT These two bits will serve as MSB of the Pre-scale
# Register to extend functional system clock frequency range.
#

# =====================================================================
# LED Driver Clock Pre-scale Register (LEDDBR)
# LEDDBR can be written through LED Control Bus. It will combine the
# LEDDCR0 [1:0] as MSB to form a 10 bits binary number to generate
# time period equivalent to 64 kHz. From here, 125 Hz or 250 Hz
# refresh rate will be generated depends on the LEDDCR0 [6] selection.
# Register Value N = Fsys/64 kHz - 1

# =====================================================================
# LED Driver ON Time Register (LEDDONR)
# LEDDONR can be written through LED Control Bus. 
# The blink ON time could be set from 0 to 8.16 seconds, with 0.032
# seconds incremental step. he actual blink ON time could be
# calculated by using the formula below. Also all available blink ON
# time options are shown in the table following the formula.
# Blink ON Time = 0.032 * NON (Sec)

# =====================================================================
# LED Driver OFF Time Register (LEDDOFR)
# LEDDOFR can be written through LED Control Bus. 
# The blink OFF time could be set from 0 to 8.16 seconds, with 0.032
# seconds incremental step. The actual blink OFF time could be
# calculated by using the formula below. Also all available blink OF
# time options are shown in the table following the formula.
# Blink OFF Time = 0.032 * NOFF (Sec)

# =====================================================================
# LED Driver Breathe ON Control Register (LEDDBCRR)
# LEDDBCRR can only be written through the LED Control Bus.
# 7 Breathe ON Enable Breathe ON Enable Bit — This bit enables the breathe ON feature setup in bit[5] and bit [3:0].
#  0 = The Breathe control if disabled; NO Breathe ON
#  1 = The breathe control is enabled
#
# 6 Breathe Edge Breathe Edge Selection Bit — This bit enables the
# breathe ON control present in this byte beapplied for both breathe
# ON and OFF
#  0 = The breathe control in this byte only be applied for ON ramp.
#  1 = The Breathe control in this byte will be applied for both ON and OFF ramp.

# 5 Breathe Mode Breathe Mode Select Bit — This bit selects the
# breathe ON/OFF mode. If this bit is cleared, the LED Driver with
# breathe ON/[OFF] with fix rate set in bit [3:0] for all colors; If
# this bit is set, the LED Driver will breathe ON/[OFF] using
# modulated rate based on the its destination brightness level.
#  0 = Unique rate for breathe ON/[OFF]
#  1 = Modulate rate for breathe ON/[OFF], based on the destination brightness level.
#
# 4 RSVD -
#
# 3:0 Breathe ON Rate User setup of the breathe ON/[OFF] rate.
# 4’b0000 = No Breathe ON/[OFF]
# The modulated ramp rate could be achieved by 16 bit counter which
# will increase on every flick rate cycle (125Hz) with the step size
# internally calculated based on the formula below:
# Nstep = 256/(UI + 1) * Brightness / 16
# During the ramp up, the PWM engine will take the MSB 8 bits of the
# 16 bit counter as input. This will result the ramp size of (of 255)
# increment per flick rate cycle (125 Hz). The effective breathe-on
# ramp rates are shown in the figure below

# =====================================================================
# LED Driver Breathe OFF Control Register (LEDDBCFR)
# LEDDBCFR can only be written through the LED Control Bus.

# 7 Breathe OFF Enable Breathe OFF Enable Bit — This bit enables the
# breathe OFF feature setup in bit [5] and bit
# [3:0]. This bit will be overridden by LEDDBCRR [7] if the LEDDBCRR [6] is set.
#  0 = The Breathe OFF control if disabled; NO Breathe OFF
#  1 = The breathe OFF control is enabled
#
# 6 PWM Range Extend
# PWM Range Extend — This bit extend the original 255/256 PWM pulse width for the linear
# counter mode to 256/256 to provide constant on PWM output for testing.
#  0 = PWM extension OFF
#  1 = Extend the PWM pulse with from 255/256 to 256/256 for Linear Counter Mode.
#
# 5 Breathe Mode Breathe Mode Select Bit — This bit selects the
# breathe ON/OFF mode. If this bit is cleared, the LED Driver with
# breathe ON/[OFF] with fix rate set in bit [3:0] for all colors; If
# this bit is set, the LED Driver will breathe ON/[OFF] using
# modulated rate based on the its destination brightness level. This
# bit will be overridden by LEDDBCRR [5] if the LEDDBCRR [6] is set.
#  0 = Unique rate for breathe OFF
#  1 = Modulate rate for breathe OFF, based on the destination brightness level.
#
# 4 RSVD -
#
# 3:0 Breathe OFF Rate
# User setup of the breathe OFF rate.
# 4’b0000 = No Breathe OFF
# Opposite to the ramp on period, the modulated ramp rate could be
# achieved by 16 bit counter which will decrease on every flick rate
# cycle (125 Hz) with the step size internally calculated based on the
# formula below:
# Nstep = 256/(UI + 1) * Brightness / 16
# During the ramp up, the PWM engine will take the MSB 8 bits of the
# 16 bit counter as input. This will result the ramp size of (of 255)
# increment per flick rate cycle (125 Hz)

# =====================================================================
# LED Driver RED Pulse Width Register (LEDDPWRR)
# LEDDPWRR can only be written through System Bus
# The LEDDPWRR allow user to setup the brightness of the RED LED
# through Pulse Width Modulation (PWM) withtotal 256 brightness
# level. Based on the PWR value, the modulated pulse with could be
# generated from 0 to 100% of the flick rate cycle in 1/256% per
# step. The Active Duty Cycle could be calculated as:
# ADC(%) = PWR/256
# In Linear Counter Mode (Non-LSFSR Mode), if the LEDDBCFR[6] bit is
# set, with PWR = 8HFF setting, the Active Duty Cycle of the PWM
# output will be 100% instead of 255/256%. This way we could provide
# constant on PWM output for characterization and validation testing.

