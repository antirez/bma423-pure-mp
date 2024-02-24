from machine import SoftI2C, Pin
import time, bma423

i2c = SoftI2C(scl=11,sda=10)
sensor = bma423.BMA423(i2c)

while True:
    print("(x,y,z),device_temperature =",
        sensor.get_xyz(),
        sensor.get_temperature())
    time.sleep(.1)
