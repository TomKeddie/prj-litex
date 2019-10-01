#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <uart.h>
#include <generated/csr.h>
#include <generated/mem.h>

// 25ms
#define TIMER_INTERVAL (CONFIG_CLOCK_FREQUENCY/40)

volatile uint8_t timer0_fired = 0;

static void timer0_isr(void) {
    if (timer0_ev_pending_read() != 0) {
        timer0_ev_pending_write(1);
        timer0_fired = 1;
    }
}

void isr(void);
void isr(void)
{
	unsigned int irqs;
	
	irqs = irq_pending() & irq_getmask();
#ifdef CSR_UART_BASE    
	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();
#endif    
    if (irqs & (1 << TIMER0_INTERRUPT))
        timer0_isr();
}

int main(void)
{
	irq_setmask(0);
	irq_setie(1);
#ifdef CSR_UART_BASE    
	uart_init();
#endif    
	timer0_en_write(0);
	timer0_reload_write(TIMER_INTERVAL);
	timer0_load_write(TIMER_INTERVAL);
	irq_setmask(irq_getmask() | (1 << TIMER0_INTERRUPT));
	timer0_en_write(1);
    
#ifdef CSR_UART_BASE    
	puts("Stub firmware booting...\n");
#endif
    const uint8_t pwm_period = 0xff;
    
    // disable
    pwm_upper_pwm_enable_write(0);
    pwm_lower_pwm_enable_write(0);
    pwm_left_pwm_enable_write(0);
    pwm_right_pwm_enable_write(0);
    // 12MHz / 100 = 120kHz
    pwm_upper_pwm_divider_write(100);
    pwm_lower_pwm_divider_write(100);
    pwm_left_pwm_divider_write(100);
    pwm_right_pwm_divider_write(100);
    // 8 bits (max)
    pwm_upper_pwm_period_write(pwm_period);
    pwm_lower_pwm_period_write(pwm_period);
    pwm_left_pwm_period_write(pwm_period);
    pwm_right_pwm_period_write(pwm_period);
    // enable
    pwm_upper_pwm_enable_write(1);
    pwm_lower_pwm_enable_write(1);
    pwm_left_pwm_enable_write(1);
    pwm_right_pwm_enable_write(1);

#if 1
    uint8_t counter = 0;
    uint8_t countup = 1;
    timer0_ev_enable_write(1);
    while(1) {
        if (timer0_fired) {
            timer0_fired = 0;
            pwm_upper_pwm_width_write(counter);
            pwm_lower_pwm_width_write(pwm_period-counter);
            pwm_left_pwm_width_write(counter);
            pwm_right_pwm_width_write(counter);
            if (countup) {
                if (counter < pwm_period-4) {
                    counter+=4;
                } else {
                    countup = 0;
                    counter-=4;
                }
            } else {
                if (counter != 0) {
                    counter-=4;
                } else {
                    countup = 1;
                    counter +=4;
                }
            }
        }
	}

#else
    while(1) {};
#endif

	return 0;
}
