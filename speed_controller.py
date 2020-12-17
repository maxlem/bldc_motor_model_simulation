import bldc_motor_model_kk
import pi_controller
import json
import numpy as np
import matplotlib.pyplot as plt

import time


class Speed_Controller:
    def __init__(self, speed_controller_data):
        bldc_motor_data    = speed_controller_data["bldc_motor"]
        pi_controller_data = speed_controller_data["pi_controller"]

        self.bldc_motor    = bldc_motor_model_kk.BLDC_Motor(bldc_motor_data)
        self.pi_controller = pi_controller.PI_Controller(pi_controller_data)


    def simulation(self, dt, iterations, u_dict, **kwargs):
        # Prepare arrays for signals
        theta_m = np.zeros(iterations)
        w_m = np.zeros(iterations)
        i_a = np.zeros(iterations)
        i_b = np.zeros(iterations)
        i_c = np.zeros(iterations)
        T_e = np.zeros(iterations)
        v_a = np.zeros(iterations)
        v_b = np.zeros(iterations)
        v_c = np.zeros(iterations)
        v_s = np.zeros(iterations)

        # Simulation main loop
        for i in range(iterations):
            if "show" in kwargs.keys() and kwargs["show"]:
                if (i+1)%1000 == 0:
                    print(str(((i+1)/iterations)*100)+"%")

            # Feedback loop sum block
            if i == 0:
                e = u_dict["w_ref"][i]
            else:
                e = u_dict["w_ref"][i] - w_m[i-1]
                
            # PI controller block
            u_dict_pi_controller = {"e": np.array([e])}
            y_dict_pi_controller = self.pi_controller.simulation_euler_anti_windup(dt, 1, u_dict_pi_controller)
            v_s[i] = y_dict_pi_controller["y"][0]

            # BLDC motor
            u_dict_bldc_motor = {"v_s": np.array([v_s[i]]), 
                                 "T_l": np.array([u_dict["T_l"][i]])}

            y_dict_dc_motor = self.bldc_motor.simulation_euler(dt, 1, u_dict_bldc_motor)

            # Load data to arrays
            theta_m[i] = y_dict_dc_motor["w_m"][0]
            w_m[i] = y_dict_dc_motor["w_m"][0]
            i_a[i] = y_dict_dc_motor["i_a"][0]
            i_b[i] = y_dict_dc_motor["i_b"][0]
            i_c[i] = y_dict_dc_motor["i_c"][0]
            v_a[i] = y_dict_dc_motor["v_a"][0]
            v_b[i] = y_dict_dc_motor["v_b"][0]
            v_c[i] = y_dict_dc_motor["v_c"][0]
            T_e[i] = y_dict_dc_motor["T_e"][0]

        # Prepare output dict
        y_dict = {
            "theta_m": theta_m,        
            "w_m": w_m,        
            "i_a": i_a,                         # \
            "i_b": i_b,                         # - plot
            "i_c": i_c,                         # /
            "v_a": v_a,                         # - plot
            "v_b": v_b,                         # - plot
            "v_c": v_c,                         # - plot
            "T_e": T_e,                         # - plot
            "V_s": v_s,                         # - plot
        }
        
        return y_dict


if __name__ == "__main__":
    # Simulation data
    simulation_data_path     = "./simulation_data.json"
    simulation_data          = json.load(open(simulation_data_path))
    position_controller_data = simulation_data["position_controller"]
    speed_controller_data    = position_controller_data["speed_controller"]

    dt = 0.01
    iterations = 10000
    t = np.ones(iterations)
    t = t*dt
    t = np.cumsum(t)
    t = t-dt
    t = t

    # Simulation input
    u_dict = {
        "w_ref" : np.append(10*np.ones(int(iterations/5)), 
                    np.append(50*np.ones(int(iterations/5)), 
                    np.append(30*np.ones(int(iterations/5)), 
                    np.append(60*np.ones(int(iterations/5)), 80*np.ones(int(iterations/5)))))),
        "T_l" : np.append(0.00*np.ones(int(iterations/4)), 
                    np.append(0.025*np.ones(int(iterations/4)), 
                    np.append(0.01*np.ones(int(iterations/4)), 
                    0.03*np.ones(int(iterations/4)))))
    }

    # Run simulation
    speed_controller  = Speed_Controller(speed_controller_data)
    simulation_output = speed_controller.simulation(dt, iterations, u_dict)

    # Plot speed
    ylim_coeff = 1.2
    plt.plot(t, simulation_output["V_s"],  label="Vs[V]")
    plt.plot(t, u_dict["w_ref"], "-r", label="ωref[rad/s]")
    plt.plot(t, simulation_output["w_m"], "-b", label="ωm[rad/s]")
    plt.plot(t, u_dict["T_l"]*1000, "-k", label="TL[mNm]")
    plt.xlim([0, t[-1]])
    plt.ylim([np.min(simulation_output["w_m"])*ylim_coeff, np.max(simulation_output["w_m"])*ylim_coeff])
    plt.xlabel("t[s]")
    plt.ylabel("ω[rad/s]")
    plt.legend(loc="right")
    plt.grid(which="both")
    plt.minorticks_on()
    plt.show()
