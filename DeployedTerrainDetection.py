from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep_ms, localtime
from bno08x import BNO08X, BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE
import ustruct
import math

PWM_ADDR = 0x40
MUX_ADDR = 0x70
MUX_IMU_CH = 3
MUX_PWM_CH = 0
PWM_BASE = 1500
LOG_DURATION_MS = 10_000
LOG_RATE_HZ = 100
PID_KP = 500.0
MOTOR_MAP = [(0,1,False), (2,3,True), (4,5,True), (6,7,False)]

def classify_terrain(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
    if gyro_z <= 0.01:
        if gyro_z <= -0.01:
            if acc_z <= 12.85:
                return 'concrete_asphalt'
            else:
                return 'grass_dirt'
        else:
            if gyro_x <= 0.02:
                return 'equilibrium'
            else:
                return 'grass_dirt'
    else:
        if acc_x <= -0.32:
            if acc_x <= -0.71:
                return 'grass_dirt'
            else:
                return 'grass_dirt'
        else:
            if gyro_y <= 0.51:
                return 'concrete_asphalt'
            else:
                return 'grass_dirt'

i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000)

class TCA9548A:
    def __init__(self, i2c, addr=0x70):
        self.i2c = i2c
        self.addr = addr
        self.last = None
    def select(self, ch):
        if ch != self.last:
            self.i2c.writeto(self.addr, bytearray([1 << ch]))
            self.last = ch
            sleep_ms(5)

mux = TCA9548A(i2c)

class PCA9685:
    def __init__(self, i2c, addr=PWM_ADDR):
        self.i2c, self.addr = i2c, addr
        self._write(0x00, 0x00)
    def _write(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytearray([val]))
    def freq(self, hz):
        prescale = int(25000000 / (4096 * hz) - 1)
        self._write(0x00, 0x10)
        self._write(0xFE, prescale)
        self._write(0x00, 0xA1)
    def duty(self, ch, val):
        val = max(0, min(4095, int(val)))
        data = ustruct.pack("<HH", 0, val)
        self.i2c.writeto_mem(self.addr, 0x06 + 4 * ch, data)

class PID:
    def __init__(self, kp): self.kp = kp
    def update(self, error): return self.kp * error

btn = Pin(21, Pin.IN, Pin.PULL_UP)

mux.select(MUX_PWM_CH)
pca = PCA9685(i2c)
pca.freq(100)
for a, b, _ in MOTOR_MAP:
    pca.duty(a, 0)
    pca.duty(b, 0)

mux.select(MUX_IMU_CH)
imu = BNO08X(i2c)
imu.enable_feature(BNO_REPORT_ACCELEROMETER)
imu.enable_feature(BNO_REPORT_GYROSCOPE)

run_idx = 1
date_str = "{:04d}{:02d}{:02d}".format(*localtime()[:3])

while True:
    print(f"Press USER button to start {run_idx}")
    while btn.value():
        sleep_ms(20)
    print(f"button pressed {run_idx}")

    f = open(f"TerrainDetection_{date_str}_run{run_idx}.csv", "w")
    f.write("time_ms,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,yaw_error,predicted_label\n")
    pid = PID(PID_KP)
    start = ticks_ms()
    interval = int(1000 / LOG_RATE_HZ)

    while ticks_diff(start + LOG_DURATION_MS, ticks_ms()) > 0:
        t = ticks_ms()
        mux.select(MUX_IMU_CH)

        ax, ay, az = imu.acc
        gx, gy, gz = imu.gyro

        pred_label = classify_terrain(ax, ay, az, gx, gy, gz)
        print(f"Predicted: {pred_label}")

        yaw_error = -gz
        correction = pid.update(yaw_error)
        left_pwm  = PWM_BASE - correction
        right_pwm = PWM_BASE + correction

        mux.select(MUX_PWM_CH)
        for i, (a, b, rev) in enumerate(MOTOR_MAP):
            pwm = left_pwm if i in [0,3] else right_pwm
            if rev:
                pca.duty(a, 0)
                pca.duty(b, pwm)
            else:
                pca.duty(a, pwm)
                pca.duty(b, 0)

        f.write(f"{t},{ax:.3f},{ay:.3f},{az:.3f},{gx:.3f},{gy:.3f},{gz:.3f},{yaw_error:.3f},{pred_label}\n")

        dt = interval - (ticks_ms() - t)
        if dt > 0:
            sleep_ms(dt)

    mux.select(MUX_PWM_CH)
    for a, b, _ in MOTOR_MAP:
        pca.duty(a, 0)
        pca.duty(b, 0)

    f.close()
    print(f"Run {run_idx} complete, TerrainDetection_{date_str}_run{run_idx}.csv")
    run_idx += 1
