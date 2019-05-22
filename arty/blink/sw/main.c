#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <uart.h>
#include <time.h>
#include <generated/csr.h>
#include <generated/mem.h>

void isr(void);
void isr(void)
{
	unsigned int irqs;
	
	irqs = irq_pending() & irq_getmask();
	
	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();
}

int main(void)
{
	irq_setmask(0);
	irq_setie(1);
	uart_init();
    time_init();
    
	puts("Stub firmware booting...\n");

    uint32_t led = 0;
    timer0_ev_enable_write(1);
    while(1) {
        if (timer0_ev_pending_read() != 0) {
            timer0_ev_pending_write(1);
            leds_out_write(led);
            led ^= 1;
        }
	}

	return 0;
}
