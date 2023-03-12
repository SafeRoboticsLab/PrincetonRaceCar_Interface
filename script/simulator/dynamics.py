import numpy as np

class Bicycle4D:
    def __init__(self, dt) -> None:
        self.dt  = dt
        self.wheelbase = 0.257
        self.control_limit = np.array([[-5, 5], [-0.35, 0.35]])
        self.state_limits = np.array([[-np.inf, np.inf],
									[-np.inf, np.inf],
									[0, 10],
									[-np.inf, np.inf]])

        
    def deriv(self, state, control, noise_sigma):
        '''
        compute the continuous time dynamics of the bicycle model
        State: [x, y, v, psi]
        Control: [accel, delta]
        '''
        _, _, v, psi = np.clip(state, self.state_limits[:, 0], self.state_limits[:, 1])
        a, delta = np.clip(control, self.control_limit[:, 0], self.control_limit[:, 1])+np.random.randn(2)*noise_sigma
        # a, delta = control
        dx = v * np.cos(psi)
        dy = v * np.sin(psi)
        dv = a
        dpsi = v * np.tan(delta) / self.wheelbase
        
        return np.array([dx, dy, dv, dpsi])
    
    def integrate(self, state, control, noise_sigma):
        '''
        integrate the continuous time dynamics of the bicycle model
        '''
        k1 = self.deriv(state, control, noise_sigma)
        k2 = self.deriv(state + self.dt / 2 * k1, control, noise_sigma)
        k3 = self.deriv(state + self.dt / 2 * k2, control, noise_sigma)
        k4 = self.deriv(state + self.dt * k3, control, noise_sigma)
        new_state = state + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        new_state[3] = np.mod(new_state[3]+np.pi, 2 * np.pi) - np.pi
        return np.clip(new_state, self.state_limits[:, 0], self.state_limits[:, 1])
    
class DoubleIntegrator:
    def __init__(self, dt) -> None:
        # State: [x, v_x, y, v_y]
        # Control: [a_x, a_y]
        self.dt = dt
        self.control_limit = np.array([[-5, 5], [-3, 3]])
        self.state_limits = np.array([[-np.inf, np.inf],
                                        [0, 3],
                                        [-np.inf, np.inf]])
        
        self.A = np.array([[1, self.dt, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, self.dt],
                            [0, 0, 0, 1]])
        
        self.B = np.array([[0, 0],
                            [self.dt, 0],
                            [0, 0],
                            [0, self.dt]])
        
    def integrate(self, state, control, noise_sigma):
        '''
        integrate the continuous time dynamics of the double integrator model
        '''
        control = np.clip(control, self.control_limit[:, 0], self.control_limit[:, 1])
        new_state = self.A @ state + self.B @ (control + np.random.randn(2)*noise_sigma)
        return np.clip(new_state, self.state_limits[:, 0], self.state_limits[:, 1])
        
        
        