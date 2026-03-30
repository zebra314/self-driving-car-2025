import numpy as np

class KalmanFilter:
    def __init__(self, x=0, y=0, yaw=0):
        # State [x, y, yaw]
        self.state = np.array([x, y, yaw])

        # Transition matrix
        self.A = np.identity(3)
        self.B = np.identity(3)

        # State covariance matrix
        self.S = np.identity(3) * 1

        # Observation matrix
        self.C = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])

        # State transition error
        self.R = np.identity(3) * 0.001

        # Measurement error
        self.Q = np.identity(2) * 1000

    def predict(self, u):
        # raise NotImplementedError

        # Predict the next state
        # (3, 1) = (3, 3) @ (3, 1) + (3, 3) @ (3, 1)
        self.state = self.A @ self.state + self.B @ u

        # Predict the next state covariance
        # (3, 3) = (3, 3) @ (3, 3) @ (3, 3) + (3, 3)
        self.S = self.A @ self.S @ self.A.T + self.R

        return self.state, self.S


    def update(self, z):
        # raise NotImplementedError

        # Compute the Kalman gain
        # (3, 2) = (3, 3) @ (3, 2) @ ((2, 3) @ (3, 3) @ (3, 2) + (2, 2))^-1
        #        = (3, 3) @ (3, 2) @ (2, 2)^-1
        K = self.S @ self.C.T @ np.linalg.inv(self.C @ self.S @ self.C.T + self.Q)

        # Compute the measurement residual
        # (2, 1) = (2, 1) - (2, 3) @ (3, 1)
        y = z - self.C @ self.state

        # Update the state
        # (3, 1) = (3, 1) + (3, 2) @ (2, 1)
        self.state = self.state + K @ y

        # Update the state covariance
        # (3, 3) = ((3, 3) - (3, 2) @ (2, 3)) @ (3, 3)
        self.S = (np.identity(3) - K @ self.C) @ self.S

        return self.state, self.S
