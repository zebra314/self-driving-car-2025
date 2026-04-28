import numpy as np
from math import cos, sin, atan2, pi

def normalize_angle(angle):
    """Normalize angle to the range [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle

class ExtendedKalmanFilter:
    def __init__(self, x0, y0, yaw0):
        # Define what state to be estimate
        # Ex.
        #   only pose -> np.array([x, y, yaw])
        #   with velocity -> np.array([x, y, yaw, vx, vy, vyaw])
        #   etc...
        self.pose = np.array([[0.0], [0.0], [0.0]])

        # Transition matrix
        self.A = np.identity(3)
        self.B = np.identity(3)

        # State covariance matrix
        self.S = np.identity(3) * 1

        # Observation matrix
        # mapping [x, y, yaw] to [x, y] measurement)
        self.C = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])

        # State transition error
        self.R = np.identity(3) * 0.1

        # Measurement error
        self.Q = np.identity(2) * 1

        print("Initialize Kalman Filter")

    def predict(self, u):
        # Base on the Kalman Filter design in Assignment 4
        # Implement a linear or nonlinear motion model for the control input
        # Calculate Jacobian matrix of the model as self.A

        dx, dy, dyaw = u[0], u[1], u[2]
        yaw = self.pose[2, 0]

        # Non-linear motion model
        self.pose[0, 0] += dx * cos(yaw) - dy * sin(yaw)
        self.pose[1, 0] += dx * sin(yaw) + dy * cos(yaw)
        self.pose[2, 0] = normalize_angle(self.pose[2, 0] + dyaw)

        # Jacobian A (w.r.t state)
        self.A = np.array([
            [1.0, 0.0, -dx * sin(yaw) - dy * cos(yaw)],
            [0.0, 1.0,  dx * cos(yaw) - dy * sin(yaw)],
            [0.0, 0.0, 1.0]
        ])

        # Covariance prediction
        self.S = self.A @ self.S @ self.A.T + self.R

    def update(self, z):
        # Base on the Kalman Filter design in Assignment 4
        # Implement a linear or nonlinear observation matrix for the measurement input
        # Calculate Jacobian matrix of the matrix as self.C

        z = np.array(z).reshape(2, 1)

        # Predicted measurement
        z_hat = self.C @ self.pose

        # Innovation
        y = z - z_hat

        # Kalman Gain: K = S * C^T * (C * S * C^T + Q)^-1
        S_inv = np.linalg.inv(self.C @ self.S @ self.C.T + self.Q)
        K = self.S @ self.C.T @ S_inv

        # State update
        self.pose = self.pose + K @ y
        self.pose[2, 0] = normalize_angle(self.pose[2, 0])

        # Covariance update
        I = np.identity(3)
        self.S = (I - K @ self.C) @ self.S

        return self.pose, self.S



