import bldc_motor_model_kk
import pi_controller
import json
import numpy as np
import matplotlib.pyplot as plt

import time


class Simulation:
    def __init__(self, bldc_motor_data, pi_controller_data, pi_controller_p_data):
        self.bldc_motor      = bldc_motor_model_kk.BLDC_Motor(bldc_motor_data)
        self.pi_controller_p = pi_controller.PI_Controller(pi_controller_p_data)
        self.pi_controller   = pi_controller.PI_Controller(pi_controller_data)

        # Simulation parameters
        self.dt = 0.01
        self.iterations = 10000
        t = np.ones(self.iterations)
        t = t*self.dt
        t = np.cumsum(t)
        t = t-self.dt
        self.t = t

        # self.w_ref = np.append(40*np.ones(int(self.iterations/2)), 40*np.ones(int(self.iterations/2)))
        # self.w_ref = np.append(10*np.ones(int(self.iterations/5)), 
        #             np.append(50*np.ones(int(self.iterations/5)), 
        #             np.append(30*np.ones(int(self.iterations/5)), 
        #             np.append(60*np.ones(int(self.iterations/5)), 80*np.ones(int(self.iterations/5))))))

        self.theta_m_ref = np.append(20*np.ones(int(self.iterations/5)), 
                    np.append(40*np.ones(int(self.iterations/5)), 
                    np.append(30*np.ones(int(self.iterations/5)), 
                    np.append(100*np.ones(int(self.iterations/5)), 80*np.ones(int(self.iterations/5))))))

        self.T_l   = np.append(0.00*np.ones(int(self.iterations/4)), 
                    np.append(0.04*np.ones(int(self.iterations/4)), 
                    np.append(0.045*np.ones(int(self.iterations/4)), 
                    0.05*np.ones(int(self.iterations/4)))))
        self.T_l   = np.zeros(self.iterations)

        self.simulation_output = {}


    def run_simulation(self, noise=False):
        self.theta_m = np.zeros(self.iterations)
        self.w_ref = np.zeros(self.iterations)
        self.w_m = np.zeros(self.iterations)
        self.i_a = np.zeros(self.iterations)
        self.i_b = np.zeros(self.iterations)
        self.i_c = np.zeros(self.iterations)
        self.T_e = np.zeros(self.iterations)
        self.v_a = np.zeros(self.iterations)
        self.v_b = np.zeros(self.iterations)
        self.v_c = np.zeros(self.iterations)
        self.v_s = np.zeros(self.iterations)

        # Simulation main loop
        for i in range(self.iterations):
            if i%1000 == 0:
                print(str((i/self.iterations)*100)+"%")

            # Feedback loop sum block
            if i == 0:
                ep = self.theta_m_ref[i]
            else:
                ep = self.theta_m_ref[i] - self.theta_m[i-1]
                
            # PI controller block
            u_dict_p = {"e": np.array([ep])}
            y_dict_pi_controller_p = self.pi_controller_p.simulation_euler_anti_windup(self.dt, 1, u_dict_p)
            self.w_ref[i] = y_dict_pi_controller_p["y"][0]

            

            # Feedback loop sum block
            if i == 0:
                e = self.w_ref[i]
            else:
                e = self.w_ref[i] - self.w_m[i-1]
                
            # PI controller block
            u_dict = {"e": np.array([e])}
            y_dict_pi_controller = self.pi_controller.simulation_euler_anti_windup(self.dt, 1, u_dict)
            self.v_s[i] = y_dict_pi_controller["y"][0]

            # BLDC motor
            u_dict = {"v_s": np.array([self.v_s[i]]), 
                      "T_l": np.array([self.T_l[i]])}

            y_dict_dc_motor = self.bldc_motor.simulation_euler(self.dt, 1, u_dict)

            self.theta_m[i] = y_dict_dc_motor["theta_m"][0]
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

        self.simulation_output = {
            "w_m": self.w_m,        
            "theta_m": self.theta_m,        
            "i_a": self.i_a,                         # \
            "i_b": self.i_b,                         # - plot
            "i_c": self.i_c,                         # /
            "v_a": self.v_a,                         # - plot
            "v_b": self.v_b,                         # - plot
            "v_c": self.v_c,                         # - plot
            "T_e": self.T_e,                         # - plot
            "V_s": self.v_s,                         # - plot
            "w_ref": self.w_ref
        }
        
        return self.simulation_output, self.t


if __name__ == "__main__":
    # default
    bldc_motor_data_path    = "./bldc_motor_data_kk_data.json"
    pi_controller_data_path = "./pi_controller_data.json"
    pi_controller_p_data_path = "./pi_controller_data2.json"
    bldc_motor_data         = json.load(open(bldc_motor_data_path))
    pi_controller_data      = json.load(open(pi_controller_data_path))
    pi_controller_data_p    = json.load(open(pi_controller_p_data_path))

    # simulation = Simulation(bldc_motor_data, pi_controller_data, pi_controller_data_p)
    # simulation_output_def, t_meas = simulation.run_simulation(noise=False)

    # # plot speed
    # ylim_coeff = 1.2
    # # plt.plot(t_meas, simulation_output_def["V_s"], label="Vs[V]")
    # plt.plot(t_meas, simulation.theta_m_ref, "-c", label="θ_ref[rad/s]")
    # plt.plot(t_meas, simulation_output_def["theta_m"], "-k", label="θm[rad/s] Kp=50")
    # # plt.plot(t_meas, simulation_output_def["w_m"], "-b", label="ωm[rad/s]")
    # # plt.plot(t_meas, simulation_output_def["w_ref"], "-r", label="ωref[rad/s]")
    # # plt.plot(t_meas, P, "-g", label="ωm")
    # # plt.plot(t_meas, P2, "-b", label="ωm")
    # # plt.xlim([0, t_meas[-1]])
    # # plt.ylim([np.min(simulation_output_def["V_s"])*ylim_coeff, np.max(simulation.theta_m_ref)*ylim_coeff])
    # # # plt.ylim([np.min(simulation_output_def["w_m"])*ylim_coeff, np.max(simulation_output_def["w_m"])*ylim_coeff])
    # # plt.xlabel("t[s]")
    # # plt.ylabel("θ[rad]")
    # # plt.legend(loc="right")
    # # plt.grid(which="both")
    # # plt.minorticks_on()
    # # plt.show()


    # pi_controller_data_p["K_p"] = 100
    # simulation = Simulation(bldc_motor_data, pi_controller_data, pi_controller_data_p)
    # simulation_output_def, t_meas = simulation.run_simulation(noise=False)

    # # plot speed
    # ylim_coeff = 1.2
    # # plt.plot(t_meas, simulation_output_def["V_s"], label="Vs[V]")
    # # plt.plot(t_meas, simulation.theta_m_ref, "-c", label="θm_ref[rad/s]")
    # plt.plot(t_meas, simulation_output_def["theta_m"], "-b", label="θm[rad/s] Kp=100")
    # # plt.plot(t_meas, simulation_output_def["w_m"], "-b", label="ωm[rad/s]")
    # # plt.plot(t_meas, simulation_output_def["w_ref"], "-r", label="ωref[rad/s]")
    # # plt.plot(t_meas, P, "-g", label="ωm")
    # # plt.plot(t_meas, P2, "-b", label="ωm")
    # # plt.xlim([0, t_meas[-1]])
    # # plt.ylim([np.min(simulation_output_def["V_s"])*ylim_coeff, np.max(simulation.theta_m_ref)*ylim_coeff])
    # # # plt.ylim([np.min(simulation_output_def["w_m"])*ylim_coeff, np.max(simulation_output_def["w_m"])*ylim_coeff])
    # # plt.xlabel("t[s]")
    # # plt.ylabel("θ[rad]")
    # # plt.legend(loc="right")
    # # plt.grid(which="both")
    # # plt.minorticks_on()
    # # plt.show()



    # pi_controller_data_p["K_p"] = 500
    # simulation = Simulation(bldc_motor_data, pi_controller_data, pi_controller_data_p)
    # simulation_output_def, t_meas = simulation.run_simulation(noise=False)

    # # plot speed
    # ylim_coeff = 1.2
    # # plt.plot(t_meas, simulation_output_def["V_s"], label="Vs[V]")
    # plt.plot(t_meas, simulation.theta_m_ref, "-c", label="θm_ref[rad/s]")
    # plt.plot(t_meas, simulation_output_def["theta_m"], "-r", label="θm[rad/s] Ti=0.1")
    # # plt.plot(t_meas, simulation_output_def["w_m"], "-b", label="ωm[rad/s]")
    # # plt.plot(t_meas, simulation_output_def["w_ref"], "-r", label="ωref[rad/s]")
    # # plt.plot(t_meas, P, "-g", label="ωm")
    # # plt.plot(t_meas, P2, "-b", label="ωm")
    # plt.xlim([0, t_meas[-1]])
    # plt.ylim([np.min(simulation_output_def["V_s"])*ylim_coeff, np.max(simulation.theta_m_ref)*ylim_coeff])
    # # plt.ylim([np.min(simulation_output_def["w_m"])*ylim_coeff, np.max(simulation_output_def["w_m"])*ylim_coeff])
    # # plt.xlabel("t[s]")
    # # plt.ylabel("θ[rad]")
    # # plt.legend(loc="upper left")
    # # plt.grid(which="both")
    # # plt.minorticks_on()
    # # plt.show()




    pi_controller_data_p["K_p"] = 500
    pi_controller_data_p["T_i"] = 0.5
    simulation = Simulation(bldc_motor_data, pi_controller_data, pi_controller_data_p)
    simulation_output_def, t_meas = simulation.run_simulation(noise=False)

    # plot speed
    ylim_coeff = 1.2
    plt.plot(t_meas, simulation_output_def["V_s"], label="Vs[V]")
    plt.plot(t_meas, simulation.theta_m_ref, "-c", label="θm_ref[rad/s]")
    plt.plot(t_meas, simulation_output_def["theta_m"], "-y", label="θm[rad/s]")
    plt.plot(t_meas, simulation_output_def["w_ref"], "-r", label="ωref[rad/s]")
    plt.plot(t_meas, simulation_output_def["w_m"], "-b", label="ωm[rad/s]")
    # plt.plot(t_meas, simulation.T_l*1000, "-k", label="TL[mNm]")
    # plt.plot(t_meas, P, "-g", label="ωm")
    # plt.plot(t_meas, P2, "-b", label="ωm")
    plt.xlim([0, t_meas[-1]])
    plt.ylim([np.min(simulation_output_def["V_s"])*ylim_coeff, np.max(simulation.theta_m_ref)*ylim_coeff])
    plt.ylim([np.min(simulation_output_def["V_s"])*ylim_coeff, np.max(simulation.theta_m_ref)*ylim_coeff])
    # plt.ylim([np.min(simulation_output_def["w_m"])*ylim_coeff, np.max(simulation_output_def["w_m"])*ylim_coeff])
    plt.xlabel("t[s]")
    plt.ylabel("θ[rad]")
    plt.legend(loc="upper left")
    plt.grid(which="both")
    plt.minorticks_on()
    plt.show()