# BMA423 simple driver.
#
# DISCLAIMER: The BMA423 chip has tons of features. In this driver the
# goal is not to support everything, but the minimum set of useful things
# that most users will want to play with: raw XYZ readings and interrupts
# for steps and for reading changes. For more advanced stuff, check the
# datasheet or use a more complex driver.
#
# Copyright (C) 2024 Salvatore Sanfilippo -- All Rights Reserved
# This code is released under the MIT license
# https://opensource.org/license/mit/
#
# Written reading the specification at:
# https://www.mouser.com/datasheet/2/783/BST-BMA423-DS000-1509600.pdf

from machine import Pin

# Registers
REG_CMD = const(0x7e)       # Write there to send commands
REG_PWR_CTL = const(0x7d)   # Used to power-on the device parts
REG_PWR_CONF = const(0x73)
REG_ACC_CONF = const(0x40)  # Out data rate, bandwidth, read mode.
REG_ACC_RANGE = const(0x41) # Acceleration range selection.
REG_TEMPERATURE = const(0x22) # Temperature sensor reading: kelvin units.

# Commands for the REG_CMD register
REG_CMD_SOFTRESET = const(0xB6)

class BMA423:
    # Acceleration range can be selected among the available settings of
    # 2G, 4G, 8G and 16G. If we want to be able to measure higher
    # max accelerations, the relative precision decreases as we have
    # a fixed 12 bit reading.
    def __init__(self,i2c,*,acc_range=4,interrupt_pin=None,callback=None):
        default_addr = [0x18,0x19] # Changes depending on SDO pin
                                   # pulled to ground or V+
        self.i2c = i2c
        self.myaddr = None
        self.callback = callback
        self.interrupt_pin = interrupt_pin
        self.interrupt_pin.irq(handler=self.irq, trigger=Pin.IRQ_FALLING)

        found_devices = i2c.scan()
        print("BMA423: scan i2c bus:", [hex(x) for x in found_devices])
        for addr in default_addr:
            if addr in found_devices:
                self.myaddr = addr
                break
        if self.myaddr == None:
            raise Exception("BMA423 not found at i2c bus")
        print("BMA423: device found at ",hex(self.myaddr))

        # Initialization.
        self.set_reg(REG_PWR_CTL,0x04) # acquisition enabled, aux disabled.

        # Enable performance mode: in this mode, the data sampling happens
        # at the specified frequency interval continuously.
        acc_bwp = 0x01 # acc_perf_mode = 1, average of 2 samples.
        acc_ord = 0x08 # 100 hz sampling rate
        self.set_reg(REG_ACC_CONF,acc_ord | (acc_bwp<<4))

        # Enable power saving: when data is not being sampled, slow clock
        # is active: more delay.
        self.set_reg(REG_PWR_CONF,0x03) # adv_power_save + fifo_self_wakeup.
    
        # Enable 4G range.
        self.set_range(acc_range)

    # Set range of 2, 4 or 8 or 16g
    def set_range(self,acc_range):
        range_to_regval = {2:0,4:1,8:2,16:3}
        if not acc_range in range_to_regval:
            raise Exception(f"Invalid range {acc_range}: use 2, 4, 8, 16",
                acc_range)
        self.range = acc_range
        self.set_reg(REG_ACC_RANGE,range_to_regval[acc_range])

    # Convert the raw 12 bit number in two's complement as a signed
    # number.
    def convert_to_int12(self,raw_value):
        if not raw_value & 0x800: return raw_value
        raw_value = ((~raw_value) & 0x7ff) + 1
        return -raw_value

    # Normalize the signed 12 bit acceleration value to
    # acceleration value in "g" according to the currently
    # selected range.
    def normalize_reading(self,reading):
        return self.range / 2047 * reading

    # Return x,y,z acceleration.
    def getxyz(self):
        rawdata = self.get_reg(0x12,6)
        acc_x = (rawdata[0] >> 4) | (rawdata[1] << 4)
        acc_y = (rawdata[2] >> 4) | (rawdata[3] << 4)
        acc_z = (rawdata[4] >> 4) | (rawdata[5] << 4)
        acc_x = self.convert_to_int12(acc_x)
        acc_y = self.convert_to_int12(acc_y)
        acc_z = self.convert_to_int12(acc_z)
        acc_x = self.normalize_reading(acc_x)
        acc_y = self.normalize_reading(acc_y)
        acc_z = self.normalize_reading(acc_z)
        return (acc_x,acc_y,acc_z)

    def irq(self,pin):
        if self.callback == None:
            printf("BMA423: not handled IRQ. Pass 'callback' during initialization")
            return
        data = self.get_touch_coords()
        if data == None: return
        self.callback(data)

    # Return the single byte at the specified register
    def get_reg(self, register, count=1):
        if count == 1:
            return self.i2c.readfrom_mem(self.myaddr,register,1)[0]
        else:
            return self.i2c.readfrom_mem(self.myaddr,register,count)

    # Write 'value' to the specified register
    def set_reg(self, register, value):
        self.i2c.writeto_mem(self.myaddr,register,bytes([value]))

# Example usage and quick test to see if your device is working.
if  __name__ == "__main__":
    from machine import SoftI2C, Pin
    import time

    i2c = SoftI2C(scl=11,sda=10)
    sensor = BMA423(i2c,interrupt_pin=Pin(14,Pin.IN),callback=None)
    while True:
        print(sensor.getxyz())
        time.sleep(.1)
