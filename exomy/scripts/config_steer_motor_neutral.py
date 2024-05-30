import time
import sys
import os
import board
import busio
import yaml
from adafruit_pca9685 import PCA9685

config_filename = '../config/exomy.yaml'

def get_steering_motor_pins():
    steering_motor_pins = {}
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file, yaml.FullLoader)
    
    if 'exomy' in param_dict:
        param_dict = param_dict['exomy']
        if 'ros__parameters' in param_dict:
            param_dict = param_dict['ros__parameters']
            for param_key, param_value in param_dict.items():
                if 'pin_steer_' in str(param_key):
                    steering_motor_pins[param_key] = param_value
            return steering_motor_pins
        else:
            print('Unexpected format of exomy.yaml file! Missing ros__parameters')
            exit()
    else:
        print('Unexpected format of exomy.yaml file! Missing namespace')
        exit()

def get_steering_pwm_neutral_values():
    steering_pwm_neutral_values = {}
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file, yaml.FullLoader)

    if 'exomy' in param_dict:
        param_dict = param_dict['exomy']
        if 'ros__parameters' in param_dict:
            param_dict = param_dict['ros__parameters']
            for param_key, param_value in param_dict.items():
                if 'steer_pwm_neutral_' in str(param_key):
                    steering_pwm_neutral_values[param_key] = param_value
            return steering_pwm_neutral_values
        else:
            print('Unexpected format of exomy.yaml file! Missing ros__parameters')
            exit()
    else:
        print('Unexpected format of exomy.yaml file! Missing namespace')
        exit()

def get_position_name(name):
    position_name = ''
    if '_fl' in name:
        position_name = 'Front Left'
    elif '_fr' in name:
        position_name = 'Front Right'
    elif '_cl' in name:
        position_name = 'Center Left'
    elif '_cr' in name:
        position_name = 'Center Right'
    elif '_rl' in name:
        position_name = 'Rear Left'
    elif '_rr' in name:
        position_name = 'Rear Right'
    return position_name

def update_config_file(steering_pwm_neutral_dict):
    output = ''
    with open(config_filename, 'rt') as file:
        for line in file:
            for key, value in steering_pwm_neutral_dict.items():
                if key in line:
                    line = line.replace(line.split(': ', 1)[1], str(value) + '\n')
                    break
            output += line

    with open(config_filename, 'w') as file:
        file.write(output)

if __name__ == "__main__":
    print(
        '''
$$$$$$$$\                     $$\      $$\           
$$  _____|                    $$$\    $$$ |          
$$ |      $$\   $$\  $$$$$$\  $$$$\  $$$$ |$$\   $$\ 
$$$$$\    \$$\ $$  |$$  __$$\ $$\$$\$$ $$ |$$ |  $$ |
$$  __|    \$$$$  / $$ /  $$ |$$ \$$$  $$ |$$ |  $$ |
$$ |       $$  $$<  $$ |  $$ |$$ |\$  /$$ |$$ |  $$ |
$$$$$$$$\ $$  /\$$\ \$$$$$$  |$$ | \_/ $$ |\$$$$$$$ |
\________|\__/  \__| \______/ \__|     \__| \____$$ |
                                           $$\   $$ |
                                           \$$$$$$  |
                                            \______/ 
        '''
    )
    print(
        '''
This script helps you to set the neutral pwm values for the steering motors.
You will iterate over all steering motors and set them to a neutral position.
The determined value is written to the config file.

Commands:
a - Decrease value for current pin
d - Increase value for current pin
q - Finish setting value for current pin

[Every of these commands must be confirmed with the enter key]

ctrl+c - Exit script
------------------------------------------------------------------------------
        '''
    )

    if not os.path.exists(config_filename):
        print("exomy.yaml does not exist. Finish config_motor_pins.py to generate it.")
        exit()

    # Create the I2C bus interface
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create the PCA9685 class instance
    pca = PCA9685(i2c)
    # Set the PWM frequency to 50Hz
    pca.frequency = 50

    # The cycle is the inverted frequency converted to milliseconds
    cycle = 1.0 / 50.0 * 1000.0  # ms

    # The time the pwm signal is set to on during the duty cycle
    on_time = 1.5  # ms

    # Duty cycle is the percentage of a cycle the signal is on
    duty_cycle = on_time / cycle

    # The PCA 9685 board requests a 16-bit number for the duty_cycle
    initial_value = int(duty_cycle * 65535.0)

    # Get all steering pins
    steering_motor_pins = get_steering_motor_pins()
    pwm_neutral_dict = get_steering_pwm_neutral_values()

    # Iterating over all motors and fine tune the zero value
    for pin_name, pin_value in steering_motor_pins.items():
        pwm_neutral_name = pin_name.replace('pin_steer_', 'steer_pwm_neutral_')
        pwm_neutral_value = pwm_neutral_dict[pwm_neutral_name]

        print('Set ' + get_position_name(pin_name) + ' steering motor: \n')
        while True:
            # Set motor
            pca.channels[pin_value].duty_cycle = pwm_neutral_value
            time.sleep(0.1)
            print('Current value: ' + str(pwm_neutral_value) + '\n')
            input_str = input(
                'q-set / a-decrease pwm neutral value/ d-increase pwm neutral value\n')
            if input_str == 'q':
                print('PWM neutral value for ' + get_position_name(pin_name) + ' has been set.\n')
                break
            elif input_str == 'a':
                print('Decreased pwm neutral value')
                pwm_neutral_value -= 5
            elif input_str == 'd':
                print('Increased pwm neutral value')
                pwm_neutral_value += 5
        pwm_neutral_dict[pwm_neutral_name] = pwm_neutral_value

    update_config_file(pwm_neutral_dict)
    print("Finished configuration!!!")
