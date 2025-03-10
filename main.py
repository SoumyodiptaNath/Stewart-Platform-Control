import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation
from Ball_Plate_System_Controller import SMC, LQR


# Global Params
ball_plate_params = {"g_acc": 9.81,
                     "dt": 1e-2}
    
smc_params = {"S_n1": 3.85,
              "S_n2": 4.40,
              "Sigma_K": 1.95,
              "Sigma_delta": 1.5}

lqr_params = {"Q": np.diag([6., 1., 4.]),
              "R": 7.5}

    
    
def simulate_controller(ref_state, ctrl_name, sim_time = 2*np.pi):
    if ctrl_name == 'smc':
        smc_params["ref_state"] = ref_state
        controller = SMC(**smc_params, **ball_plate_params)
    
    if ctrl_name == 'lqr':
        lqr_params["ref_state"] = ref_state
        controller = LQR(**lqr_params, **ball_plate_params)
    
    err_history = []
    pos_history = []
    angles_history = []
    ref_pos_history = []
    
    while controller.t < sim_time:
        controller.get_ctrl_input()
        controller.step_sim()
        
        err_history.append(np.copy(controller.err[1, :]))
        angles_history.append(np.copy(controller.plate_state))
        pos_history.append(np.copy(controller.ball_state[0,:]))
        ref_pos_history.append(np.copy(controller.curr_ref_state[0,:]))
        
    return (np.array(pos_history), np.squeeze(np.array(angles_history)),
            np.array(err_history), np.array(ref_pos_history))



class animate_simulation():
    def __init__(self, pos_history, angles_history, err_history, 
                 ref_pos_history, dt=ball_plate_params["dt"]):
        
        self.err_history = err_history
        self.pos_history = pos_history
        self.angles_history = angles_history
        self.ref_pos_history = ref_pos_history
        self.dt = dt
        
        self.lines = []
        self.fig = plt.figure(figsize = (15, 7))
        gs = GridSpec(2, 2, figure = self.fig)
        
        ax = self.fig.add_subplot(gs[:, 0])
        ax.set(xlim=[1.1*np.min(self.pos_history), 1.1*np.max(self.pos_history)],
               ylim=[1.1*np.min(self.pos_history), 1.1*np.max(self.pos_history)],
               xlabel='X (m)', ylabel='Y [m]')
        
        self.lines.append(ax.plot(self.pos_history[0, 0], self.pos_history[0, 1])[0])
        self.lines.append(ax.plot(self.ref_pos_history[0, 0], self.ref_pos_history[0, 1])[0])
        ax.legend(["Actual", "Reference"])
        
        ax = self.fig.add_subplot(gs[0, 1])
        ax.set(xlim=[0, 2*np.pi], xlabel='Time (s)', ylabel='Angles (radians)',
               ylim=[1.1*np.min(angles_history), 1.1*np.max(angles_history)])
        
        self.lines.append(ax.plot(0., self.angles_history[0, 0])[0])
        self.lines.append(ax.plot(0., self.angles_history[0, 1])[0])
        ax.legend(['Alpha', 'Beta'])
        
        ax = self.fig.add_subplot(gs[1, 1])
        ax.set(xlim=[0, 2*np.pi], xlabel='Time (s)', ylabel='Pos Error (m)',
               ylim=[1.1*np.min(err_history), 1.1*np.max(err_history)])
        
        self.lines.append(ax.plot(0., self.err_history[0, 0])[0])
        self.lines.append(ax.plot(0., self.err_history[0, 1])[0])
        ax.legend(['X-Xd', 'Y-Yd'])
        
        
    def update_frame(self, curr_time):
        time_span = np.arange(0, curr_time)*self.dt
        
        self.lines[0].set_data(self.pos_history[:curr_time, 0], self.pos_history[:curr_time, 1])
        self.lines[1].set_data(self.ref_pos_history[:curr_time, 0], self.ref_pos_history[:curr_time, 1])
        
        self.lines[2].set_data(time_span, self.angles_history[:curr_time, 0])
        self.lines[3].set_data(time_span, self.angles_history[:curr_time, 1])
        
        self.lines[4].set_data(time_span, self.err_history[:curr_time, 0])
        self.lines[5].set_data(time_span, self.err_history[:curr_time, 1])
            
        return self.lines
    
    
    def animate_plot(self, anim_flag=1):
        if anim_flag:
            anim = FuncAnimation(fig = self.fig, func = self.update_frame,
                                 frames = self.pos_history.shape[0],
                                 interval = self.dt*1e3, repeat = False)
        else:
            self.update_frame(self.pos_history.shape[0])
            
        plt.show()



def main():
    ref_state = lambda theta : np.array([[np.cos(theta), np.sin(theta)],
                                         [-np.sin(theta), np.cos(theta)]])*7.5e-2
    
    data = simulate_controller(ref_state, 'smc')
    Anim = animate_simulation(*data)
    Anim.animate_plot(anim_flag = 1)
    
    
if __name__ == "__main__":
    main()