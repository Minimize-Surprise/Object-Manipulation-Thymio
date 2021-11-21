# HOW TO:
#
# Trim the potentiometer till the printed value is at its lowest point.
# Then turn it up slowly while the value sill remains at its lowest point.
# If it switches back to a higher number again you've hit the sweet spot.
# Enjoy your force resistor experience!
#
# @Christian Charles

import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
DEBUG = 0

# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
SPICLK = 18

# SPI port on the ADC to the Cobbler
SPIMISO = 23
SPIMOSI = 24
SPICS = 25
# set up the SPI interface pins
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)


# def readadc(adcnum, clockpin, mosipin, misopin, cspin):
def readadc(num):
    adcnum = num
    clockpin = SPICLK
    mosipin = SPIMOSI
    misopin = SPIMISO
    cspin = SPICS
    if (adcnum > 7) or (adcnum < 0):
        return -1
    GPIO.output(cspin, True)

    GPIO.output(clockpin, False)  # start clock low
    GPIO.output(cspin, False)  # bring CS low

    commandout = adcnum
    commandout |= 0x18  # start bit + single-ended bit
    commandout <<= 3  # we only need to send 5 bits here
    for i in range(5):
        if commandout & 0x80:
            GPIO.output(mosipin, True)
        else:
            GPIO.output(mosipin, False)
        commandout <<= 1
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)

    adcout = 0
    # read in one empty bit, one null bit and 10 ADC bits
    for i in range(12):
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)
        adcout <<= 1
        if GPIO.input(misopin):
            adcout |= 0x1

    GPIO.output(cspin, True)

    adcout >>= 1  # first bit is 'null' so drop it
    return adcout


if __name__ == '__main__':
    while True:
        # hang out and do nothing for a half second
        time.sleep(0.1)
        print(str(readadc(0)))
