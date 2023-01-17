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

        
    def deriv(self, state, control):
        '''
        compute the continuous time dynamics of the bicycle model
        State: [x, y, v, psi]
        Control: [accel, delta]
        '''
        _, _, v, psi = np.clip(state, self.state_limits[:, 0], self.state_limits[:, 1])
        a, delta = np.clip(control, self.control_limit[:, 0], self.control_limit[:, 1])+np.random.randn(2)*0.1
        # a, delta = control
        dx = v * np.cos(psi)
        dy = v * np.sin(psi)
        dv = a
        dpsi = v * np.tan(delta) / self.wheelbase
        
        return np.array([dx, dy, dv, dpsi])
    
    def integrate(self, state, control):
        '''
        integrate the continuous time dynamics of the bicycle model
        '''
        k1 = self.deriv(state, control)
        k2 = self.deriv(state + self.dt / 2 * k1, control)
        k3 = self.deriv(state + self.dt / 2 * k2, control)
        k4 = self.deriv(state + self.dt * k3, control)
        new_state = state + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        new_state[3] = np.mod(new_state[3]+np.pi, 2 * np.pi) - np.pi
        return np.clip(new_state, self.state_limits[:, 0], self.state_limits[:, 1])