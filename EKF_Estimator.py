import numpy as np

class CellEKF:
    def __init__(self, dt, C_Ah, Rs, Cp, Rp, Q, R):
        self.dt = dt  # Sample time
        self.C_Ah = C_Ah
        self.Rs = Rs
        self.Cp = Cp
        self.Rp = Rp
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance

        # State vector initialization
        self.x_hat = np.array([[0],  # SoC
                               [0],  # Vp
                               [0]]) # R0
        
        # State estimation error covariance initialization
        self.P = np.eye(3)

    def set_initial_state(self, SoC, Vp, R0):
        self.x_hat = np.array([[SoC],
                               [Vp],
                               [R0]])

    def voc(self, soc):
        # TODO: Map SoC to open circuit voltage
        return 3.7

    def f(self, x, i):
        return np.array([
            [-i / self.C_Ah],
            [i / self.Cp - x[1, 0] / (self.Rp * self.Cp)],
            [0]
        ])
    
    def h(self, x, i):
        # Placeholder for h(x,i) function
        # For demonstration, using Vt directly. Replace with actual function.
        return self.voc(x[0,0]) - i*self.Rs - x[1, 0]
    
    def F_d(self):
        return np.array([
            [1, 0, 0],
            [0, np.exp(self.dt / (self.Rp * self.Cp)), 0],
            [0, 0, 1]
        ])
    
    def H_d(self, x, i):
        dVoc_dSoC = 1  # Placeholder, TODO: Replace with soc voc map
        return np.array([[dVoc_dSoC, -1, -i]])
    
    def predict(self, i):
        # Project the states ahead (a priori estimate)
        self.x_hat = self.f(self.x_hat, i)
        # Project the error covariance ahead
        Fd = self.F_d()
        self.P = Fd @ self.P @ Fd.T + self.Q
    
    def correct(self, Vt, i):
        # Compute Kalman gain
        H = self.H_d(self.x_hat, i)
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.R)
        # Update the estimate with the measurement Vt (a posteriori estimate)
        self.x_hat = self.x_hat + K * (Vt - self.h(self.x_hat, i))
        # Update the error covariance
        self.P = (np.eye(3) - K @ H) @ self.P

    def update(self, Vt, i):
        self.predict(i)
        self.correct(Vt, i)
        # Return estimated SoC
        return self.x_hat[0, 0]
    
class PackEKF:
    def __init__(self, ns, np, dt, C_Ah, Rs, Cp, Rp, Q, R):
        self.ns = ns    # Number of series cells
        self.np = np    # Number of parallel cells
        self.dt = dt  # Sample time
        self.C_Ah = C_Ah
        self.Rs = Rs
        self.Cp = Cp
        self.Rp = Rp
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance

        # State vector initialization
        self.x_hat = np.array([[0],  # SoC
                               [3.7],  # Vp
                               [0]]) # R0
        
        # State estimation error covariance initialization
        self.P = np.eye(3)
