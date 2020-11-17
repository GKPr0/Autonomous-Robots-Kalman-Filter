import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['CONSTANT_SPEED'] = False

class KalmanFilter:
    def __init__(self):
        self.v = 0
        self.prev_time = 0
        # Initial State
        self.x = np.matrix([[0.],
                            [0.]])

        # Uncertainity Matrix
        self.P = np.matrix([[1000, 0.],
                            [0., 1000]])

        # Next State Function
        self.F = np.matrix([[1, 0],
                            [0., 1]])

        # Measurement Function
        self.H = np.matrix([[1., 0.]])

        # Measurement Uncertainty
        self.R = np.matrix([[0.01]])

        # Identity Matrix
        self.I = np.matrix([[1., 0.],
                            [0., 1.]])
    def predict(self,t):
        self.update_dt(t)

        # Calculate updated state vector
        self.x = self.F * self.x

        self.P =  self.F * self.P * np.transpose(self.F)
        print("P = {}".format(self.P))
        return self.x[0,0]
    def measure_and_update(self,measurements,t):
        self.update_dt(t)

        Z = np.matrix(measurements)
        y = np.transpose(Z) - self.H * self.x
        S = self.H * self.P * np.transpose(self.H) + self.R
        print(self.H * self.P)
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)

        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P
        
        # Add some uncertainty to P, so system wont be so condfident
        # This will increase reaction speed in trade of to accuraccy
        self.P[0,0] += 0.4
        self.P[1,1] += 0.4

        self.v = self.x[1,0]
        self.prev_time = t
        return

    def update_dt(self, t):
        dt = t - self.prev_time
        self.F[0,1] = dt


sim_run(options,KalmanFilter)
