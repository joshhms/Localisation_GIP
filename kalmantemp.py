import numpy as np

X_INDEX = 2
Y_INDEX = 3
Z_INDEX = 1

class KalmanFilter:
   
    def __init__(self):

        # MPU constants
        self.gyro_scale_factor = 131
        self.pitch_rate_offset = 0.507735
        self.roll_rate_offset  = 0.053538
        self.yaw_rate_offset   = -1.502070
        self.rad2deg = 57.2958

        # Encoder
        self.encoder = 0

        # Sensor data
        self.AccX = 0.0 
        self.AccY = 0.0 
        self.AccZ = 0.0
        self.GyroX = 0.0 
        self.GyroY = 0.0 
        self.GyroZ = 0.0 
        self.temperature = 0.0

        # Accelerometer calibration
        self.accel_scale_factor = 16384
        self.ax_scale  = 1.000865
        self.ax_offset = 0.003108
        self.ay_scale  = 0.982406
        self.ay_offset = 0.034223
        self.az_scale  = 0.997780
        self.az_offset = -0.019510

        # Timing
        self.dt = 0.004

        # Kalman state
        self.x = np.zeros((4,1))
        self.z = np.zeros((4,1))
        self.y = np.zeros((4,1))

        self.P = np.eye(4)

        self.Q = np.matrix([[(self.dt**4)/4, 0.0, (self.dt**3)/2, 0.0], 
                            [0.0, self.dt**4 / 4, 0.0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0.0, (self.dt**2), 0.0],
                            [0.0, (self.dt**3)/2, 0.0, (self.dt**2)]])  

        self.R = np.matrix([[0.057759, 0.000279, 0.0, 0.0],
                            [0.000279, 0.035079, 0.0, 0.0],
                            [0.0, 0.0, 0.14484, 0.00451],
                            [0.0, 0.0, 0.00451, 0.008457]])

        self.F = np.matrix([[1.0, 0.0, self.dt, 0.0], 
                            [0.0, 1.0, 0.0, self.dt],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]]) 

        self.H = np.eye(4)
        self.I = np.eye(4)

        self.K = np.zeros((4,4))
        self.inv = np.zeros((4,4))

        self.yaw_angle = 0.0

        # Adaptive tuning
        self.epsilon = np.matrix('0.0')
        self.epsilon_max = 9.5
        self.Q_scale_factor = 3
        self.count = 0

    # -----------------------------
    # Measurement model
    # -----------------------------
    def get_z(self):

        ax_g = self.AccX / self.accel_scale_factor
        ay_g = self.AccY / self.accel_scale_factor
        az_g = self.AccZ / self.accel_scale_factor

        ax = self.ax_scale * ax_g + self.ax_offset
        ay = self.ay_scale * ay_g + self.ay_offset
        az = self.az_scale * az_g + self.az_offset

        total = np.sqrt(ax**2 + ay**2 + az**2)

        if abs(ay) < total:
            self.z[0] = np.asin(ay / total) * self.rad2deg

        if abs(ax) < total:
            self.z[1] = np.asin(ax / total) * -self.rad2deg

        self.z[2] = (self.GyroX / self.gyro_scale_factor) - self.roll_rate_offset
        self.z[3] = (self.GyroY / self.gyro_scale_factor) - self.pitch_rate_offset

    # -----------------------------
    # Assign new data
    # -----------------------------
    def assign_new_sensor_readings(self, data, temp_c, encoder):

        self.AccX = data[X_INDEX]
        self.AccY = data[Y_INDEX]
        self.AccZ = data[Z_INDEX]

        self.GyroX = data[X_INDEX + 3]
        self.GyroY = data[Y_INDEX + 3]
        self.GyroZ = data[Z_INDEX + 3]

        self.temperature = temp_c
        self.encoder = encoder

        self.get_z()

    # -----------------------------
    # Kalman steps
    # -----------------------------
    def get_prediction(self):
        self.x = (self.F * self.x)
        self.P = (self.F * self.P * self.F.T) + self.Q

    def get_kalman_gain(self):
        self.inv = ((self.H * self.P * self.H.T) + self.R).I
        self.K = (self.P * self.H.T) * self.inv

    def get_update(self):
        self.x = self.x + (self.K * (self.z - (self.H * self.x)))
        self.P = (self.I - (self.K * self.H)) * self.P
        self.update_yaw()

    def get_residual(self):
        self.y = self.z - self.x

    def get_epsilon(self):
        self.epsilon = (self.y.T * (self.inv * self.y))

    def scale_Q(self):
        if self.epsilon[0] > self.epsilon_max:
            self.Q *= self.Q_scale_factor
            self.count += 1
        elif self.count > 0:
            self.Q /= self.Q_scale_factor
            self.count -= 1

    def update_yaw(self):
        self.yaw_angle += self.dt * ((self.GyroZ / self.gyro_scale_factor) - self.yaw_rate_offset)

    # -----------------------------
    # Main loop
    # -----------------------------
    def kalmanloop(self, data, temp_c, encoder):

        self.get_prediction()
        self.assign_new_sensor_readings(data, temp_c, encoder)

        self.get_kalman_gain()
        self.get_update()

        self.get_residual()
        self.get_epsilon()
        self.scale_Q()

        return [
            data[0],
            self.x.item(0),
            self.z.item(0),
            self.x.item(1),
            self.z.item(1),
            self.yaw_angle,
            self.AccX,
            self.AccY,
            self.AccZ,
            self.z.item(2),
            self.z.item(3),
            (self.GyroZ / self.gyro_scale_factor) - self.yaw_rate_offset,
            self.temperature,
            self.encoder
        ]