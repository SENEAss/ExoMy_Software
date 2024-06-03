import time
import os
import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from shutil import copyfile

DRIVE_MOTOR, STEER_MOTOR = [0, 1]

pos_names = {
    1: 'fl',
    2: 'fr',
    3: 'cl',
    4: 'cr',
    5: 'rl',
    6: 'rr',
}

pin_dict = {}

# Create the I2C bus interface
i2c = busio.I2C(board.SCL, board.SDA)

# Create the PCA9685 class instance
pca = PCA9685(i2c)
# Set the PWM frequency to 50Hz
pca.frequency = 50

# Configure the pulse range for your servo (adjust min_pulse and max_pulse as needed)
min_pulse = 500
max_pulse = 2400

class Motor():
    def __init__(self, pin):
        self.pin_name = 'pin_'
        self.pin_number = pin
        self.servo = servo.Servo(pca.channels[pin], min_pulse=min_pulse, max_pulse=max_pulse)

    def wiggle_motor(self):
        # Set the motor to two different positions
        self.servo.angle = 0
        time.sleep(1.0)
        self.servo.angle = 180
        time.sleep(1.0)
        self.servo.angle = 90
        time.sleep(0.5)
        self.servo.angle = None  # Stop the motor

    def stop_motor(self):
        self.servo.angle = None  # Turn the motor off

def print_exomy_layout():
    print(
        '''
        1 fl-||-fr 2
             ||
        3 cl-||-cr 4
        5 rl====rr 6
        '''
    )

def update_config_file():
    file_name = '../config/exomy.yaml'
    template_file_name = file_name + '.template'

    if not os.path.exists(file_name):
        copyfile(template_file_name, file_name)
        print("exomy.yaml.template was copied to exomy.yaml")

    output = ''
    with open(file_name, 'rt') as file:
        for line in file:
            for key, value in pin_dict.items():
                if key in line:
                    line = line.replace(line.split(': ', 1)[1], str(value) + '\n')
                    break
            output += line

    with open(file_name, 'w') as file:
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
###############
Motor Configuration

This scripts leads you through the configuration of the motors.
First we have to find out, to which pin of the PWM board a motor is connected.
Look closely which motor moves and type in the answer.

Ensure to run the script until the end, otherwise your changes will not be saved!
This script can always be stopped with ctrl+c and restarted.
All other controls will be explained in the process.
###############
        '''
    )

    for pin_number in range(16):
        motor = Motor(pin_number)
        motor.stop_motor()

    for pin_number in range(16):
        motor = Motor(pin_number)
        motor.wiggle_motor()
        type_selection = ''
        while True:
            print("Pin #{}".format(pin_number))
            print(
                'Was it a steering or driving motor that moved, or should I repeat the movement? ')
            type_selection = input('(d)rive (s)teer (r)epeat - (n)one (f)inish_configuration\n')
            if type_selection == 'd':
                motor.pin_name += 'drive_'
                print('Good job\n')
                break
            elif type_selection == 's':
                motor.pin_name += 'steer_'
                print('Good job\n')
                break
            elif type_selection == 'r':
                print('Look closely\n')
                motor.wiggle_motor()
            elif type_selection == 'n':
                print('Skipping pin')
                break
            elif type_selection == 'f':
                print('Finishing calibration at pin {}.'.format(pin_number))
                break
            else:
                print('Input must be d, s, r, n, or f\n')

        if type_selection == 'd' or type_selection == 's':
            while True:
                print_exomy_layout()
                pos_selection = input(
                    'Type the position of the motor that moved.[1-6] or (r)epeat\n')
                if pos_selection == 'r':
                    print('Look closely\n')
                else:
                    try:
                        pos = int(pos_selection)
                        if 1 <= pos <= 6:
                            motor.pin_name += pos_names[pos]
                            break
                        else:
                            print('The input was not a number between 1 and 6\n')
                    except ValueError:
                        print('The input was not a number between 1 and 6\n')

            pin_dict[motor.pin_name] = motor.pin_number
            print('Motor set!\n')
            print('########################################################\n')
        elif type_selection == 'f':
            break

    print('Now we will step through all the motors and check whether they have been assigned correctly.\n')
    print('Press ctrl+c if something is wrong and start the script again. \n')

    for pin_name in pin_dict:
        print('moving {}'.format(pin_name))
        print_exomy_layout()

        pin = pin_dict[pin_name]
        motor = Motor(pin)
        motor.wiggle_motor()
        input('Press button to continue')

    print("You assigned {}/12 motors.".format(len(pin_dict.keys())))

    print('Write to config file.\n')
    update_config_file()
    print(
        '''
    $$$$$$$$\ $$\           $$\           $$\                       $$\ 
    $$  _____|\__|          \__|          $$ |                      $$ |
    $$ |      $$\ $$$$$$$\  $$\  $$$$$$$\ $$$$$$$\   $$$$$$\   $$$$$$$ |
    $$$$$\    $$ |$$  __$$\ $$ |$$  _____|$$  __$$\ $$  __$$\ $$  __$$ |
    $$  __|   $$ |$$ |  $$ |$$ |\$$$$$$\  $$ |  $$ |$$$$$$$$ |$$ /  $$ |
    $$ |      $$ |$$ |  $$ |$$ | \____$$\ $$ |  $$ |$$   ____|$$ |  $$ |
    $$ |      $$ |$$ |  $$ |$$ |$$$$$$$  |$$ |  $$ |\$$$$$$$\ \$$$$$$$ |
    \__|      \__|\__|  \__|\__|\_______/ \__|  \__| \_______| \_______|
                                                                        
    '''
    )

pca.deinit()
