# BLDC motor model

#### Technologies
Project was created and tested with:
* Windows 10
* Python 3.6.5


#### Description
Project containing scripts for brushless three-phase dc motor (BLDC motor) simulation. There are scripts for sole BLDC motor simulation, PI controller with anti-windup system simulation but the most important one is simulation.py script that perform closed-loop controll of BLDC motor with PI controller.


#### Setup
- Run following block of commands in bldc_motor_model_simulation\ catalog:
```
python -m virtualenv venv
cd venv
cd Scripts
activate
cd ..
cd ..
pip install -r requirements.txt
```
- Set all parameters in bldc_motor_model_kk_data.json
- Set all parameters in pi_controller_data.json
- Set input signals, time step and numberof iterations in Simulation class in simulation.py script


#### Run
Go to bldc_motor_model_simulation\ and run command:
```
python simulation.py
```


#### References
This project is created mainly based on following documents:
- Stefán Baldursson - "BLDC Motor Modelling and Control – A Matlab®/Simulink®Implementation", May, 2005,Institutionen för Energi och Miljö
- Krzysztof Krykowski - "Silniki PM BLDC właściwości, sterowanie, aplikacje"