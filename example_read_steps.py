from machine import SoftI2C, Pin
import time, bma423

i2c = SoftI2C(scl=11,sda=10)
sensor = bma423.BMA423(i2c)

# Enable steps counting
sensor.load_features_config()
sensor.enable_features_detection("step-count")

print("####################################")
print("WARNING: you need to bump the device for a long time as the counter is not updated in real time. Then you will see 8, 10, 12 steps and so forth.")
print("####################################")
while True:
    print("steps:", sensor.get_steps())
    time.sleep(1)
