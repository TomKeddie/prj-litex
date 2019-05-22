#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <uart.h>
#include <generated/csr.h>
#include <generated/mem.h>

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

	puts("Stub firmware booting...\n");

	while(1) {
	}

	return 0;
}
