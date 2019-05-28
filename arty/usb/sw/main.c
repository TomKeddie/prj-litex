#include <stdio.h>
#include <irq.h>
#include <uart.h>
#include <../../../../foboot/sw/include/usb.h>
#include <time.h>
#include <../../../../foboot/sw/include/dfu.h>
#include <../../../../foboot/sw/include/rgb.h>
#include <../../../../foboot/sw/include/spi.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include <system.h>

struct ff_spi *spi;

void isr(void);
void isr(void)
{
	unsigned int irqs;
	
	irqs = irq_pending() & irq_getmask();
	
	if(irqs & (1 << UART_INTERRUPT))
		uart_isr();
}

void msleep(int ms) {
        timer0_en_write(0);
        timer0_reload_write(0);
        timer0_load_write(SYSTEM_CLOCK_FREQUENCY/1000*ms);
        timer0_en_write(1);
        timer0_update_value_write(1);
        while(timer0_value_read()) timer0_update_value_write(1);
}

void sleep(unsigned int seconds);
void sleep(unsigned int seconds) {
    msleep(seconds*1000);
}

void rgb_mode_error(void) {
    rgb_out_write(1); // red
}

void rgb_mode_writing(void) {
    rgb_out_write(2); // blue
}

void rgb_mode_done(void) {
    rgb_out_write(6); // green/blue
}

void rgb_mode_idle(void) {
    rgb_out_write(4);
}

static void riscv_reboot_to(void *addr, uint32_t boot_config) {
    reboot_addr_write((uint32_t)addr);

    // If requested, just let USB be idle.  Otherwise, reset it.
    if (boot_config & 0x00000020) // NO_USB_RESET
        usb_idle();
    else
        usb_disconnect();

    // Figure out what mode to put SPI flash into.
    if (boot_config & 0x00000001) { // QPI_EN
        spiEnableQuad();
        picorvspi_cfg3_write(picorvspi_cfg3_read() | 0x20);
    }
    if (boot_config & 0x00000002) // DDR_EN
        picorvspi_cfg3_write(picorvspi_cfg3_read() | 0x40);
    if (boot_config & 0x00000002) // CFM_EN
        picorvspi_cfg3_write(picorvspi_cfg3_read() | 0x10);
    rgb_mode_error();

    // Vexriscv requires three extra nop cycles to flush the cache.
    if (boot_config & 0x00000010) { // FLUSH_CACHE
        asm("fence.i");
        asm("nop");
        asm("nop");
        asm("nop");
    }

    // Reset the Return Address, zero out some registers, and return.
    asm volatile(
        "mv ra,%0\n\t"    /* x1  */
        "mv sp,zero\n\t"  /* x2  */
        "mv gp,zero\n\t"  /* x3  */
        "mv tp,zero\n\t"  /* x4  */
        "mv t0,zero\n\t"  /* x5  */
        "mv t1,zero\n\t"  /* x6  */
        "mv t2,zero\n\t"  /* x7  */
        "mv x8,zero\n\t"  /* x8  */
        "mv s1,zero\n\t"  /* x9  */
        "mv a0,zero\n\t"  /* x10 */
        "mv a1,zero\n\t"  /* x11 */

        // /* Flush the caches */
        // ".word 0x400f\n\t"
        // "nop\n\t"
        // "nop\n\t"
        // "nop\n\t"

        "ret\n\t"

        :
        : "r"(addr)
    );
}

void reboot(void) {
    irq_setie(0);
    irq_setmask(0);

    uint32_t reboot_addr = dfu_origin_addr();
    uint32_t boot_config = 0;

    // Free the SPI controller, which returns it to memory-mapped mode.
    spiFree();

    // Scan for configuration data.
    int i;
    int riscv_boot = 1;
    uint32_t *destination_array = (uint32_t *)reboot_addr;
    for (i = 0; i < 32; i++) {
        // Look for FPGA sync pulse.
        if ((destination_array[i] == 0x7e99aa7e)
         || (destination_array[i] == 0x7eaa997e)) {
            riscv_boot = 0;
            break;
        }
        // Look for "boot config" word
        else if (destination_array[i] == 0xb469075a) {
            boot_config = destination_array[i + 1];
        }
    }

    if (riscv_boot) {
        riscv_reboot_to((void *)reboot_addr, boot_config);
    }
    else {
        // Issue a reboot
        warmboot_to_image(2);
    }
    __builtin_unreachable();
}

static void init(void)
{
    spi = spiAlloc();
    spiSetPin(spi, SP_MOSI, 0);
    spiSetPin(spi, SP_MISO, 1);
    spiSetPin(spi, SP_WP, 2);
    spiSetPin(spi, SP_HOLD, 3);
    spiSetPin(spi, SP_CLK, 4);
    spiSetPin(spi, SP_CS, 5);
    spiSetPin(spi, SP_D0, 0);
    spiSetPin(spi, SP_D1, 1);
    spiSetPin(spi, SP_D2, 2);
    spiSetPin(spi, SP_D3, 3);
    spiInit(spi);
    spiFree();
#if 0    
    maybe_boot_fbm();
#endif
    spiInit(spi);
    irq_setmask(0);
    irq_setie(1);
    uart_init();
    usb_init();
    dfu_init();
    time_init();

}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

 	puts("USB init...\n");
    init();

 	puts("USB booting...\n");

    usb_connect();
    puts("0\n");

    uint32_t led = 0;
    timer0_ev_enable_write(1);
    puts("1\n");
    while (1)
    {
        puts("2\n");
        usb_poll();
        dfu_poll();
        if (timer0_ev_pending_read() != 0) {
            timer0_ev_pending_write(1);
            usbled_out_write(led);
            led ^= 1;
        }
    }
    return 0;
}
