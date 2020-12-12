import numpy as np








global_state = [0,0,0] #globalny stan - x, y, theta
l = 5


def matr(state):
    return np.transpose(np.array([
        [np.cos(state[2]),np.sin(state[2]), l], 
        [np.cos(state[2]),np.sin(state[2]),-l]])) # to sie mnozy przez wektor wejśc

N = 15000


dt = 0.01
Vl = np.linspace(0, 100, N)
Vr = np.linspace(0, 100, N)

inputy = np.array([Vl, Vr])  #<--- predkosci silnikow, to wychodzi z modelu , trzeba dorzucic krok

# integral
for i in range(N):
    global_state += matr(global_state)@inputy[:,i]*dt

print(global_state)