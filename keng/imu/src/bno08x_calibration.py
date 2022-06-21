import time
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = I2C(1) # Device is /dev/i2c-1
time.sleep(1)
bno = BNO08X_I2C(i2c, debug=False)

bno.hard_reset()
bno.initialize()

bno.begin_calibration()
# TODO: UPDATE UART/SPI
bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)

startT = time.time()
print("calibrating...")

while True:
    stopT = time.time()
    if stopT-startT >= 10:
        break

print("calibration done")
