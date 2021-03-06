#!/usr/bin/env python3
import sys
from litex.tools.litex_client import RemoteClient

UCFG_REG_GO = 0x80
UCFG_REG_ADDRMASK = 0x3F
SMSC_334x_MAP = {
    "VIDL": 0x00,
    "VIDH": 0x01,
    "PIDL": 0x02,
    "PIDH": 0x03,

    "FUNC_CTL": 0x04,
    "FUNC_CTL_SET": 0x05,
    "FUNC_CTL_CLR": 0x06,

    "INTF_CTL": 0x07,
    "INTF_CTL_SET": 0x08,
    "INTF_CTL_CLR": 0x09,

    "OTG_CTL": 0x0A,
    "OTG_CTL_SET": 0x0B,
    "OTG_CTL_CLR": 0x0C,

    "USB_INT_EN_RISE": 0x0D,
    "USB_INT_EN_RISE_SET": 0x0e,
    "USB_INT_EN_RISE_CLR": 0x0f,

    "USB_INT_EN_FALL": 0x10,
    "USB_INT_EN_FALL_SET": 0x11,
    "USB_INT_EN_FALL_CLR": 0x12,

    "USB_INT_STAT": 0x13,
    "USB_INT_LATCH": 0x14,

    "DEBUG": 0x15,

    "SCRATCH": 0x16,
    "SCRATCH_SET": 0x17,
    "SCRATCH_CLR": 0x18,

    "CARKIT": 0x19,
    "CARKIT_SET": 0x1A,
    "CARKIT_CLR": 0x1B,

    "CARKIT_INT_EN": 0x1D,
    "CARKIT_INT_EN_SET": 0x1E,
    "CARKIT_INT_EN_CLR": 0x1F,

    "CARKIT_INT_STAT": 0x20,
    "CARKIT_INT_LATCH": 0x21,

    "HS_COMP_REG":   0x31,
    "USBIF_CHG_DET": 0x32,
    "HS_AUD_MODE":   0x33,

    "VND_RID_CONV": 0x36,
    "VND_RID_CONV_SET": 0x37,
    "VND_RID_CONV_CLR": 0x38,

    "USBIO_PWR_MGMT": 0x39,
    "USBIO_PWR_MGMT_SET": 0x3A,
    "USBIO_PWR_MGMT_CLR": 0x3B,
}

def ulpiread(wb, addr):
    # assert self.__check_clkup()
    
    wb.write(0x82003014, UCFG_REG_GO | (addr & UCFG_REG_ADDRMASK))
    
    while wb.read(0x82003014) & UCFG_REG_GO:
        pass
    
    return wb.read(0x82003010)

def ulpiwrite(wb, addr, value):
    #assert self.__check_clkup()

    wb.write(0x82003008, value)
    wb.write(0x8200300c, UCFG_REG_GO | (addr & UCFG_REG_ADDRMASK))

    while wb.read(0x8200300c) & UCFG_REG_GO:
        pass

wb = RemoteClient()
wb.open()
# csr_register,ucfg_rst,0x82003000,1,rw
# csr_register,ucfg_stat,0x82003004,1,ro
# csr_register,ucfg_wdata,0x82003008,1,rw
# csr_register,ucfg_wcmd,0x8200300c,1,rw
# csr_register,ucfg_rdata,0x82003010,1,ro

# csr_register,ucfg_rcmd,0x82003014,1,rw

# led
wb.regs.led_out.write(1)

# ulpi
print("VID 0x{:02x}{:02x}".format(ulpiread(wb, 1), ulpiread(wb, 0)))
print("PID 0x{:02x}{:02x}".format(ulpiread(wb, 3), ulpiread(wb, 2)))

print("clock lock {:02x}".format(wb.read(0x82003004)))

# set to non-drive; set FS or HS as requested
speed = "ls"
if speed == "hs":
    ulpiwrite(wb, 4, 0x48)
elif speed == "fs":
    ulpiwrite(wb, 4, 0x49)
elif speed == "ls":
    ulpiwrite(wb, 4, 0x4a)
else:
    assert 0,"Invalid Speed"

wb.regs.led_out.write(0)

wb.close()

