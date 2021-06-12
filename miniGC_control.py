import time
from datetime import datetime
import os
import weakref

import board
import busio
import RPi.GPIO as GPIO

import I2C_LCD_driver
# import RaspPi_GPIO_RL_Util as rl_util

# Adafruit Sensors
import adafruit_mcp9808  # i2c sensor [temp] indoor array
import adafruit_hts221  # i2c sensor [hum/temp] outdoor env
import adafruit_sht31d  # i2c [humd/temp, weather-proofed] indoor
import adafruit_pca9685  # i2c hardware pwm board

"""
$ pip3 install adafruit-circuitpython-###
to import necessary i2c libraries for sensors
"""

class GPIOactuator:
    actuator_list = []

    def __init__(self, actuator_obj, actuation_function):

        self.actuator = actuator_obj
        self.actuation_function = actuation_function
        self.actuator_list.append(weakref.ref(self))
        self.current_setpoint = None

    def actuate(self, setpoint):
        self.current_setpoint = setpoint
        self.actuation_function(self.actuator, setpoint)


def set_pwm_output(pca_obj, duty_cycle: float):
    """Takes in PCA PWM output and duty cycle from 0-100% and converts to 0x0 - 0xffff (65535) & updates PWM output."""
    if duty_cycle > 100:
        raise ValueError('PWM Duty Cycle must be between 0.0% & 100.0%')
    pca_obj.duty_cycle = 65535 * duty_cycle / 100


def set_gpio_output(gpio_pin, output_state):
    """Sets digital output pin."""
    GPIO.output(gpio_pin, output_state)


# CONFIG vars
TEMP_UNIT = 'f'  # 'c' or 'f'
TEMP_SF = 2  # decimal place / rounding of sensor input
HUMD_SF = 2

# --- I2C Sensors Initialize ---
# init I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# sensor addresses
# Temperature - MCP9808 (top/bottom, left/right interior array position)
temp_TL_array = adafruit_mcp9808.MCP9808(i2c, address=0x18)  # pos1, left-upper rail
temp_BL_array = adafruit_mcp9808.MCP9808(i2c, address=0x19)  # pos2, left-lower rail
temp_TR_array = adafruit_mcp9808.MCP9808(i2c, address=0x1A)  # pos3, right-upper rail
temp_BR_array = adafruit_mcp9808.MCP9808(i2c, address=0x1B)  # pos4, right-lower rail
temp_peltier = adafruit_mcp9808.MCP9808(i2c, address=0x1C)  # pos5, peltier heat sink
# Humidity & Temp - SHT30-D Weatherproof
temp_humd_indoor = adafruit_sht31d.SHT31D(i2c)  # pos7, inside gc
# Humidity & Temp - HTS221 Outdoor
temp_humd_outdoor = adafruit_hts221.HTS221(i2c)  # pos8, outside gc

# --- 1W Sensor Initialize ---
#  1-wire DS18B20 digital temp sensor addresses
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
# w1 address
temp_h20 = '28-8a2017eb8dff'  # pos6, in water reservoir

# -- I2C PWM Control Initialize ---
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 120  # Hz
# PWM output
fan_L_circ = GPIOactuator(pca.channels[0], set_pwm_output)
fan_L_circ = GPIOactuator(pca.channels[1], set_pwm_output)
fan_L_vent = GPIOactuator(pca.channels[2], set_pwm_output)
fan_R_vent = GPIOactuator(pca.channels[3], set_pwm_output)
fan_mister = GPIOactuator(pca.channels[4], set_pwm_output)
fan_peltier = GPIOactuator(pca.channels[5], set_pwm_output)
pump_reservoir = GPIOactuator(pca.channels[6], set_pwm_output)

# --- GPIO pin scheme ---
GPIO.setmode(GPIO.BCM)  # BCM channel, pin #, ex: GPIO #
# Output
mister_pin = GPIOactuator(17, set_gpio_output)
GPIO.setup(mister_pin.actuator, GPIO.OUT)

peltier_power_pin = GPIOactuator(27, set_gpio_output)
GPIO.setup(peltier_power_pin.actuator, GPIO.OUTPUT)

peltier_control_pin = GPIOactuator(22, set_gpio_output)
GPIO.setup(peltier_control_pin.actuator, GPIO.OUTPUT)

# --- LDC SCREEN ---
lcd = 0x27  # I2C
lcd_cols = 20
lcd_rows = 2
mylcd = I2C_LCD_driver.lcd()

# --- TIME ---
now = datetime.now()


def get_time():
    return datetime.now().strftime("%H:%M:%S")


def convert_temp_to_f(temp_c: float):
    """Convert temperature c to f."""
    return round((temp_c * 1.8) + 32, TEMP_SF)


def actuate(action_array: list):







def convert_temp(temp_c: float):
    # converts temp in degrees C to degrees F
    temp_f = (temp_c * 1.8) + 32
    return temp_round(temp_f)


def temp_round(temp: float, res: int = TEMP_RES):
    return round(temp, res)


def humd_round(humd: float, res: int = HUMD_RES):
    return round(humd, res)


def get_w1_temp(w1_addr: str, temp_unit: str = TEMP_UNIT):
    w1_dir = '/sys/bus/w1/devices/' + w1_addr + '/w1_slave'
    w1slave_file = open(w1_dir)
    w1slave_text = w1slave_file.read()
    # parse data script
    text_2ndline = w1slave_text.split('\n')[1]
    temp_data = text_2ndline.split(' ')[9]  # 10th item
    temp_reading = float(temp_data[2:])  # ignore 't='
    temp_c = temp_reading / 1000
    if temp_unit is 'c':
        return temp_round(temp_c)
    else:
        return convert_temp(temp_c)


def get_i2c_temp(i2c_sensor, temp_unit: str = TEMP_UNIT):
    temp_c = i2c_sensor.temperature
    if temp_unit is 'c':
        return temp_round(temp_c)
    else:
        return convert_temp(temp_c)


def get_i2c_humd(i2c_sensor):
    humidity = i2c_sensor.relative_humidity
    return humd_round(humidity)


while True:
    temp_list = [get_i2c_temp(temp_TL_array), get_i2c_temp(temp_BL_array), get_i2c_temp(temp_TR_array),
                 get_i2c_temp(temp_BR_array), get_i2c_temp(temp_peltier),
                 get_i2c_temp(temp_humd_indoor), get_i2c_temp(temp_humd_outdoor), get_w1_temp(
            ds18_t_h20)]

    humidity_list = [get_i2c_humd(temp_humd_indoor), get_i2c_humd(temp_humd_outdoor)]
    print('-----------------------------------------------------')
    print('Temps:')
    print(temp_list)
    print('Humidity:')
    print(humidity_list)
    time.sleep(1)



