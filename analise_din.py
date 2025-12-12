import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from utils.toolbox1 import cinematica_nova
from utils.model_robot_rrrp import robot_DH
from utils.toolbox1 import traj_quintica

#carrega o robo1.57
meu_robo, L1, L2, L3 = robot_DH()

# posição da junta inicial e final
q_atual = np.array([0.0, 0.0, 0.0, 0.0])
q_final = np.array([np.pi/6, -np.pi/4, np.pi/1.5, 0.2]) 

# Tempo de simulacao
time_s = 5.0
dt = 0.05
steps = int(time_s / dt)
t = np.linspace(0, time_s, steps)

#cinematica:
xyz, T_final = cinematica_nova(q_final, L1, L2, L3) #carrega a matriz

#trajetoria
t_total, traj_posicoes, qd, qdd = traj_quintica(q_atual, q_final, time_s, dt)
        
#torque
tau = np.zeros((steps, 4))
for i in range(steps):
   for i in range(steps):
    tau[i, :] = meu_robo.rne(traj_posicoes[i, :], qd[i, :], qdd[i, :])

        
juntas = ['J1', 'J2', 'J3','J4']

# Posição, velocidade e aceleração
fig, axs = plt.subplots(3, 1, figsize=(8, 10))
for i in range(4):
    axs[0].plot(t, traj_posicoes[:, i], label=juntas[i])
    axs[1].plot(t, qd[:, i], label=juntas[i])
    axs[2].plot(t, qdd[:, i], label=juntas[i])

axs[0].set_ylabel("Posição [rad/m]")
axs[1].set_ylabel("Velocidade [rad/s or m/s]")
axs[2].set_ylabel("Aceleração [rad/s² or m/s²]")
for ax in axs:
    ax.set_xlabel("Tempo [s]")
    ax.grid(True)
    ax.legend()

plt.tight_layout()
plt.show()

plt.figure(figsize=(8,4))
for i in range(4):
 plt.plot(t, tau[:, i], label=juntas[i])
plt.xlabel("Tempo [s]")
plt.ylabel("Torque [Nm]")
plt.title("Torque em cada junta")
plt.grid(True)
plt.legend()
plt.show()

# 4. Simula o Movimento
meu_robo.plot(traj_posicoes, backend="pyplot", dt=dt)

        
        
