import bldc_motor_model_kk
import pi_controller
import json
import numpy as np
import matplotlib.pyplot as plt

import time


class BLDC_Motor_Driver:
    def __init__(self, bldc_motor_data, pi_controller_data):
        self.bldc_motor    = bldc_motor_model_kk.BLDC_Motor(bldc_motor_data)
        self.pi_controller = pi_controller.PI_Controller(pi_controller_data)

    def run_euler_simulation(self, dt, iterations, u_dict, noise=False, debug=False):
        self.w_m = np.zeros(iterations)
        self.i_a = np.zeros(iterations)
        self.i_b = np.zeros(iterations)
        self.i_c = np.zeros(iterations)
        self.T_e = np.zeros(iterations)
        self.v_a = np.zeros(iterations)
        self.v_b = np.zeros(iterations)
        self.v_c = np.zeros(iterations)
        self.v_s = np.zeros(iterations)

        # Simulation main loop
        for i in range(iterations):
            if debug:
                if i%1000 == 0:
                    print(str((i/iterations)*100)+"%")

            w_ref = u_dict["w_ref"][i]
            T_l   = u_dict["T_l"][i]

            # Feedback loop sum block
            if i == 0:
                e = w_ref
            else:
                e = w_ref - self.w_m[i-1]
                
            # PI controller block
            u_dict_pi_controller = {"e": np.array([e])}
            y_dict_pi_controller = self.pi_controller.simulation_euler_anti_windup(dt, 1, u_dict_pi_controller)
            self.v_s[i] = y_dict_pi_controller["y"][0]

            # BLDC motor
            u_dict_dc_motor = {"v_s": np.array([self.v_s[i]]), 
                               "T_l": np.array([T_l])}

            y_dict_dc_motor = self.bldc_motor.simulation_euler(dt, 1, u_dict_dc_motor)

            self.w_m[i] = y_dict_dc_motor["w_m"][0]
            self.i_a[i] = y_dict_dc_motor["i_a"][0]
            self.i_b[i] = y_dict_dc_motor["i_b"][0]
            self.i_c[i] = y_dict_dc_motor["i_c"][0]
            self.v_a[i] = y_dict_dc_motor["v_a"][0]
            self.v_b[i] = y_dict_dc_motor["v_b"][0]
            self.v_c[i] = y_dict_dc_motor["v_c"][0]
            self.T_e[i] = y_dict_dc_motor["T_e"][0]

            if noise:
                noise = np.random.normal(0, .1)
                self.w_m[i] += noise

        y_dict = {
            "w_m": self.w_m,        
            "i_a": self.i_a,                         # \
            "i_b": self.i_b,                         # - plot
            "i_c": self.i_c,                         # /
            "v_a": self.v_a,                         # - plot
            "v_b": self.v_b,                         # - plot
            "v_c": self.v_c,                         # - plot
            "T_e": self.T_e,                         # - plot
        }
        
        return y_dict


if __name__ == "__main__":
    bldc_motor_data_path    = "./bldc_motor_data_kk_data.json"
    pi_controller_data_path = "./pi_controller_data.json"
    bldc_motor_data         = json.load(open(bldc_motor_data_path))
    pi_controller_data      = json.load(open(pi_controller_data_path))


    # Simulation parameters
    dt = 0.001
    iterations = 10000
    t = np.ones(iterations)
    t = t*dt
    t = np.cumsum(t)
    t = t-dt


    u_dict = {
        "w_ref" : np.append(30*np.ones(int(iterations/2)), 10*np.ones(int(iterations/2))),
        "T_l"   : 0*np.ones(iterations)
    }


    bldc_motor_driver = BLDC_Motor_Driver(bldc_motor_data, pi_controller_data)
    simulation_output_def = bldc_motor_driver.run_euler_simulation(dt, iterations, u_dict, noise=False)
    # P = abs(simulation_output_def["i_a"]*simulation_output_def["v_a"])+ \
    #     abs(simulation_output_def["i_b"]*simulation_output_def["v_b"])+ \
    #     abs(simulation_output_def["i_c"]*simulation_output_def["v_c"])

    # P2 = simulation_output_def["T_e"]*simulation_output_def["w_m"]


    # plot speed
    ylim_coeff = 1.2
    plt.plot(t, u_dict["w_ref"], "-r", label="ωref")
    plt.plot(t, simulation_output_def["w_m"], "-g", label="ωm")
    # plt.plot(t_meas, P, "-g", label="ωm")
    # plt.plot(t_meas, P2, "-b", label="ωm")
    plt.xlim([0, t[-1]])
    plt.ylim([np.min(simulation_output_def["w_m"])*ylim_coeff, np.max(simulation_output_def["w_m"])*ylim_coeff])
    plt.xlabel("t[s]")
    plt.ylabel("ω[rad/s]")
    plt.legend(loc="right")
    plt.grid(which="both")
    plt.minorticks_on()
    plt.show()
