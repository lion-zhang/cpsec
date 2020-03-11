import time
from Adafruit_BNO055 import BNO055 as b


class BNO055:

    def __init__(self, poll_delay=0.01):
        # Create and configure the BNO sensor connection.  Make sure only ONE of the
        # below 'bno = ...' lines is uncommented:
        # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
        self.bno = b.BNO055(serial_port='/dev/ttyAMA0', rst=21)
        # Initialize the BNO055 and stop if something went wrong.
        if not self.bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
        # Print system status and self test result.
        status, self_test, error = self.bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # # Print out an error if system status is in error mode.
        # if status == 0x01:
        #     print('System error: {0}'.format(error))
        #     print('See datasheet section 4.3.59 for the meaning.')
        # # Print BNO055 software revision and other diagnostic data.
        # sw, bl, accel, mag, gyro = self.bno.get_revision()
        # print('Software version:   {0}'.format(sw))
        # print('Bootloader version: {0}'.format(bl))
        # print('Accelerometer ID:   0x{0:02X}'.format(accel))
        # print('Magnetometer ID:    0x{0:02X}'.format(mag))
        # print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

        self.heading = self.roll = self.pitch = self.sys = self.gyro = self.accel = self.mag = \
            self.ori_x = self.ori_y = self.ori_z = self.ori_w = self.temp_c = self.mag_x = self.mag_y = \
            self.mag_z = self.gyr_x = self.gyr_y = self.gyr_z = self.acc_x = self.acc_y = self.acc_z = \
            self.lacc_x = self.lacc_y = self.lacc_z = self.gra_x = self.gra_y = self.gra_z = 0

        self.poll_delay = poll_delay
        self.on = True
        self.vm = 0
        self.vp = 0

    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)

    def poll(self):
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        self.heading, self.roll, self.pitch = self.bno.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        self.sys, self.gyro, self.accel, self.mag = self.bno.get_calibration_status()
        # Other values you can optionally read:
        # Orientation as a quaternion:
        self.ori_x, self.ori_y, self.ori_z, self.ori_w = self.bno.read_quaternion()
        # Sensor temperature in degrees Celsius:
        self.temp_c = self.bno.read_temp()
        # Magnetometer data (in micro-Teslas):
        self.mag_x, self.mag_y, self.mag_z = self.bno.read_magnetometer()
        # Gyroscope data (in degrees per second):
        self.gyr_x, self.gyr_y, self.gyr_z = self.bno.read_gyroscope()
        # Accelerometer data (in meters per second squared):
        self.acc_x, self.acc_y, self.acc_z = self.bno.read_accelerometer()

        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        self.lacc_x, self.lacc_y, self.lacc_z = self.bno.read_linear_acceleration()
        # Gravity acceleration data (i.e. acceleration just from gravity--returned
        # in meters per second squared):
        self.gra_x, self.gra_y, self.gra_z = self.bno.read_gravity()

    def run_threaded(self):
        return self.heading, self.roll, self.pitch, self.sys, self.gyro, self.accel, self.mag, \
               self.ori_x, self.ori_y, self.ori_z, self.ori_w, self.temp_c, self.mag_x, self.mag_y, \
               self.mag_z, self.gyr_x, self.gyr_y, self.gyr_z, self.acc_x, self.acc_y, self.acc_z, \
               self.lacc_x, self.lacc_y, self.lacc_z, self.gra_x, self.gra_y, self.gra_z

    def run(self):
        self.poll()
        return self.heading, self.roll, self.pitch, self.sys, self.gyro, self.accel, self.mag, \
               self.ori_x, self.ori_y, self.ori_z, self.ori_w, self.temp_c, self.mag_x, self.mag_y, \
               self.mag_z, self.gyr_x, self.gyr_y, self.gyr_z, self.acc_x, self.acc_y, self.acc_z, \
               self.lacc_x, self.lacc_y, self.lacc_z, self.gra_x, self.gra_y, self.gra_z

    def shutdown(self):
        self.on = False


if __name__ == "__main__":
    iter = 0
    p = BNO055()
    while iter < 100:
        heading, roll, pitch, sys, gyro, accel, mag, \
        ori_x, ori_y, ori_z, ori_w, temp_c, mag_x, mag_y, \
        mag_z, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, \
        lacc_x, lacc_y, lacc_z, gra_x, gra_y, gra_z = p.run()
        print(heading, roll, pitch, sys, gyro, accel, mag, ori_x,
              ori_y, ori_z, ori_w, temp_c, mag_x, mag_y, mag_z,
              gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z, lacc_x,
              lacc_y, lacc_z, gra_x, gra_y, gra_z)
        time.sleep(0.01)
        iter += 1
