# HOW TO:
#
# Add the following under "/boot" in the "config.txt"
# "dtoverlay=i2c-gpio,i2c_gpio_sda=5,i2c_gpio_scl=6"
#
# Just use the read_sensor_value method with either 3 or 1 for the front or back facing light sensor.
# Enjoy your light sensor experience!
#
# @Christian Charles

import smbus
import time

# disable the initialisation message
show_initialisation_msg = True

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print
    "Error importing RPi.GPIO! Superuser?"


class Light_sensor:
    LED_CONTROL = 0
    LED_STATUS = False
    bus = 0

    def __init__(self, bus_number, device_address=0x29):  # CHANGED BUS NUMBER
        # --------------------- # Konfiguration ------------------ #
        # Pi-Version 1 benutzt Kanal 0
        self.KANAL = 1  # Pi-Version 2 u. 3 benutzen Kanal 1
        # -------------------------------------------------------- #
        self.LED_CONTROL = 4
        self.LED_STATUS = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.LED_CONTROL, GPIO.OUT, initial=GPIO.LOW)

        self.bus = smbus.SMBus(bus_number)
        # I2C address 0x29
        # Register 0x12 has device ver.
        # Register addresses must be OR'ed with 0x80
        self.bus.write_byte(0x29, 0x80 | 0x12)
        self.ver = self.bus.read_byte(0x29)
        # version # should be 0x44
        if self.ver == 0x44:
            self.bus.write_byte(0x29, 0x80 | 0x00)  # 0x00 = ENABLE register
            self.bus.write_byte(0x29, 0x01 | 0x02)  # 0x01 = Power on, 0x02 RGB sensors enabled
            self.bus.write_byte(0x29, 0x80 | 0x14)  # Reading results start register 14, LSB then MSB
            # bus.write(byte(0x29, 0x11|0x00)
            GPIO.output(self.LED_CONTROL, GPIO.LOW)
            if show_initialisation_msg:
                position = ""
                if bus_number == 3:
                    position = "Front facing "
                elif bus_number == 1:
                    position = "Back facing "
                print(position + "TCS34725 initialized")

    def read_sensor_value(self):
        data = self.bus.read_i2c_block_data(0x29, 0)
        clear = clear = data[1] << 8 | data[0]
        return clear


if __name__ == '__main__':

    # 3 => front facing sensor
    sensor03 = Light_sensor(3)
    # 1 => back facing sensor
    sensor01 = Light_sensor(1)

    while True:
        print(sensor03.read_sensor_value())
        print(sensor01.read_sensor_value())
        time.sleep(1)
