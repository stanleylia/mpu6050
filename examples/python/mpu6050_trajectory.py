import smbus2
import time
import math

# MPU6050 Register Map (partial)
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


class LowPassFilter:
    """Simple exponential moving average low-pass filter."""

    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.state = None

    def filter(self, value):
        if self.state is None:
            self.state = value
        else:
            self.state = self.alpha * value + (1 - self.alpha) * self.state
        return self.state


class MPU6050:
    def __init__(self, bus_id=1, address=MPU6050_ADDR):
        self.bus = smbus2.SMBus(bus_id)
        self.address = address
        # Wake up the MPU6050 as it starts in sleep mode
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0)

    def read_accel(self):
        ax = read_word(self.bus, self.address, ACCEL_XOUT_H)
        ay = read_word(self.bus, self.address, ACCEL_XOUT_H + 2)
        az = read_word(self.bus, self.address, ACCEL_XOUT_H + 4)
        return ax / 16384.0, ay / 16384.0, az / 16384.0

    def read_gyro(self):
        gx = read_word(self.bus, self.address, GYRO_XOUT_H)
        gy = read_word(self.bus, self.address, GYRO_XOUT_H + 2)
        gz = read_word(self.bus, self.address, GYRO_XOUT_H + 4)
        return gx / 131.0, gy / 131.0, gz / 131.0


def main():
    mpu = MPU6050()
    ax_filter = LowPassFilter(alpha=0.2)
    ay_filter = LowPassFilter(alpha=0.2)
    az_filter = LowPassFilter(alpha=0.2)

    vx = vy = vz = 0.0
    px = py = pz = 0.0

    last_time = time.time()
    print("Press Ctrl+C to stop")
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            ax, ay, az = mpu.read_accel()
            gx, gy, gz = mpu.read_gyro()

            ax = ax_filter.filter(ax)
            ay = ay_filter.filter(ay)
            az = az_filter.filter(az - 1.0)  # remove gravity

            vx += ax * dt
            vy += ay * dt
            vz += az * dt

            px += vx * dt
            py += vy * dt
            pz += vz * dt

            print(f"Pos[m]: x={px:.3f} y={py:.3f} z={pz:.3f}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopping")


if __name__ == "__main__":
    main()
