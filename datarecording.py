import serial
import msvcrt
import time
import csv
import datetime

from kalman import KalmanFilter

class RobotController:

    def __init__(self):
        self.save_state = False
        self.run_loop = True
        self.logfilename = 'log.csv'
        self.openfile = None

       
        self.header = [
            'timestamp',
            'roll_filtered',
            'roll_raw',
            'pitch_filtered',
            'pitch_raw',
            'yaw_raw',
            'ax', 'ay', 'az',
            'gx', 'gy', 'gz',
            'temp_c',
            'encoder'
        ]

    def keyboard_read(self):
        if msvcrt.kbhit():
            command = msvcrt.getch().decode('utf-8')
            match command:
                case 'q':
                    self.run_loop = False
                    if self.save_state:
                        self.save_state = False
                        print('quitting')
                        self.openfile.close()

                case 's':
                    if self.save_state:
                        self.save_state = False
                        print('finish saving')
                        self.openfile.close()
                    else:
                        self.save_state = True
                        print('Saving')
                        self.logfilename = f'log{datetime.datetime.now():%Y%m%d_%H%M%S}.csv'
                        self.openfile = open(self.logfilename, 'w', encoding='utf-8', newline='')
                        self.writer = csv.writer(self.openfile, quoting=csv.QUOTE_ALL)
                        self.writer.writerow(self.header)

    def main(self):
        print("Running")

        myFilter = KalmanFilter()

        serialPort = serial.Serial(
            port="COM10",
            baudrate=115200,
            bytesize=8,
            timeout=2,
            stopbits=serial.STOPBITS_ONE
        )

        count = 0

        while self.run_loop:
            serialString = serialPort.readline()

            try:
                received_string = serialString.decode("Ascii").strip()

                if not received_string:
                    continue

                if received_string[0] == "#":
                    print(received_string)
                    continue

               
                # elapsed, ax, ay, az, gx, gy, gz, temp_c, encoder
                parts = received_string.split(',')

                if len(parts) < 9:
                    continue  # skip incomplete lines

                # Parse data
                data = [int(x) for x in parts[:7]]
                temp_c = float(parts[7])
                encoder = int(parts[8])

                
                filtered = myFilter.kalmanloop(
                    data=data,
                    temp_c=temp_c,
                    encoder=encoder
                )

                
                output_row = filtered

                if self.save_state:
                    self.writer.writerow(output_row)
                    debug_output = f's, {output_row}'
                else:
                    debug_output = f'_, {output_row}'

                if count > 10:
                    count = 0
                    self.keyboard_read()
                    print(debug_output)
                else:
                    count += 1

            except Exception as e:
                print("Parse error:", e)
                pass


myController = RobotController()
myController.main()
