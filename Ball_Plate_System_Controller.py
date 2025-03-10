import numpy as np
from control import lqr


class BallPlate():
    def __init__(self, g_acc, dt):
        self.t = 0.
        self.dt = dt
        self.Const_coeff = 5*g_acc/7
        self.ball_state = np.zeros((3, 2))
        self.plate_state = np.zeros((1, 2))
        self.disturbance = np.array([1e-1, 1e-1])*((np.random.random(2)-0.5)[0])
        
    def step_sim(self):
        self.ball_state[2,:] = -(self.Const_coeff
                                 *np.sin(self.plate_state
                                         *(1+self.disturbance)))
        
        del_vel = self.ball_state[2,:]*self.dt
        self.ball_state[0,:] += (self.ball_state[2,:] + 0.5*del_vel)*self.dt
        self.ball_state[1,:] += del_vel
        self.t += self.dt


        
class SMC(BallPlate):
    def __init__(self, ref_state, S_n1, S_n2,
                 Sigma_K, Sigma_delta, **sim_params):
        
        self.ref_state_func = ref_state
        self.curr_ref_state = ref_state(0)
        
        super().__init__(**sim_params)
        self.err = np.zeros_like(self.ball_state)
        
        self.K = Sigma_K
        self.d = Sigma_delta
        self.n = np.array([S_n1, S_n2, 1])
        
    
    def K_sigma(self, s):
        return self.K * s/(np.abs(s) + self.d)
    
        
    def get_ctrl_input(self):
        self.curr_ref_state = self.ref_state_func(self.t)
        self.err[1:,:] = (self.ball_state[:2,:]
                          -self.curr_ref_state)
        
        self.err[0,:] += self.err[1,:]
        self.plate_state = ((self.n[:-1] @ self.err[1:,:]
                              +self.K_sigma(self.n @ self.err))
                            /self.Const_coeff)
    


class LQR(BallPlate):
    def __init__(self, ref_state, Q, R, **sim_params):
        
        self.ref_state_func = ref_state
        self.curr_ref_state = ref_state(0)
        
        super().__init__(**sim_params)
        self.err = np.zeros_like(self.ball_state)
        
        A = np.array([[0., 1., 0.],
                      [0., 0., 1.],
                      [0., 0., 0.]])
        
        B = np.array([[0], [0], [-self.Const_coeff]])
        
        self.K, _, _ = lqr(A, B, Q, R)
                
    
    def get_ctrl_input(self):
        self.curr_ref_state = self.ref_state_func(self.t)
        self.err[1:,:] = (self.ball_state[:2,:]
                          -self.curr_ref_state)
        
        self.err[0,:] += self.err[1,:]
        self.plate_state = -self.K @ self.err
        
    