import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]

options['DRIVE_IN_CIRCLE'] = True
# If False, measurements will be x,y.
# If True, measurements will be x,y, and current angle of the car.
# Required if you want to pass the driving in circle.
options['MEASURE_ANGLE'] = True
options['RECIEVE_INPUTS'] = True

class KalmanFilter:
    def __init__(self):

        # Initial State
        self.x = np.matrix([[0.],   # x - position
                            [0.],   # y - position
                            [0.],   # v - velocity
                            [0.],   # theta - heading
                            [0.]])  # dtheta - stering

        self.U = np.matrix([[0.],   
                            [0.],  
                            [0.],   
                            [0.],
                            [0.]]) 

        # Uncertainity Matrix
        self.P = np.matrix([[1000, 0., 0., 0., 0.],
                            [0., 1000, 0., 0., 0.],
                            [0., 0., 1000, 0., 0.],
                            [0., 0., 0., 1000, 0.],
                            [0., 0., 0., 0., 1000],])

        # Next State Function
        self.F = np.matrix([[1, 0., 1, 0., 0.],
                            [0., 1, 1, 0., 0.],
                            [0., 0., 1, 0., 0.],
                            [0., 0., 0., 1, 1.],
                            [0., 0., 0., 0., 1],])

        # Measurement Function
        self.H = np.matrix([[1., 0., 0., 0., 0.],
                            [0., 1., 0., 0., 0.],
                            [0., 0., 1., 0., 0.]])

        # Measurement Uncertainty
        self.R = np.matrix([[0.1, 0., 0.],
                            [0., 0.1, 0.],
                            [0., 0., 0.1]])

        # Identity Matrix
        self.I = np.matrix([[1, 0., 0., 0., 0.],
                            [0., 1, 0., 0., 0.],
                            [0., 0., 1, 0., 0.],
                            [0., 0., 0., 1, 0.],
                            [0., 0., 0., 0., 1],])

    def predict(self, dt):
        self.update_F(dt)

        self.x = self.F * self.x + self.U

        self.P =  self.F * self.P * np.transpose(self.F)

        return [self.x[0], self.x[1]]

    def measure_and_update(self,measurements, dt):
        self.update_F(dt)

        Z = np.matrix(measurements)
        y = np.transpose(Z) - self.H * self.x
        S = self.H * self.P * np.transpose(self.H) + self.R
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)

        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P
        
        self.add_uncertainty(0.3)

        return [self.x[0], self.x[1]]

    def recieve_inputs(self, u_steer, u_pedal):
        self.U[2] = u_pedal
        self.U[3] = u_steer

    def update_F(self,dt):
        x_vel_ratio = dt * np.cos(self.x[3])
        y_vel_ratio = dt * np.cos(self.x[3])

        self.F[0,2] = x_vel_ratio
        self.F[1,2] = y_vel_ratio
        self.F[3,4] = dt

    def add_uncertainty(self, uncertainty):
        self.P[0,0] += uncertainty
        self.P[1,1] += uncertainty
        self.P[2,2] += uncertainty
        self.P[3,3] += uncertainty
        self.P[4,4] += uncertainty


sim_run(options,KalmanFilter)
