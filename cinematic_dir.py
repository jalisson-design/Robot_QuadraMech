import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from utils.toolbox1 import cinematica_nova
from utils.model_robot_rrrp import robot_DH
from utils.model_robot_rrrp import robot_URDF
from utils.toolbox1 import traj_quintica

#carrega o robo
robo_DH, L1, L2, L3 = robot_DH()

# print(robo_DH)
# print("-" * 30)  

# posição da junta inicial e final
q_atual = np.array([0.0, 0.0, 0.0, 0.0])
# q_final = np.array([np.pi/6, -np.pi/6, np.pi/4, 0.2])
#q_final = np.array([0, 1.052, -1.052, -0.305])

# Tempo de simulacao
time_s = 5.0
dt = 0.05
steps = int(time_s / dt)
t = np.linspace(0, time_s, steps)

while True:
    print("-" * 30)
    print(f" Posição Atual Juntas: {q_atual}")
    print("Digite o ALVO para as juntas (q1 q2 q3 q4) em RADIANOS/METROS")
    print("q1, q2, q3 = Radianos, q4 = Metros")
    print("90° = 1.57, 45° = 0.785")
    print("Exemplo (90 graus na base, 0.5m de extensão): 1.57 0 0 0.5")
    entrada = input(">>> Destino (ou 'sair'): ")
    
    if entrada == "sair":
      break
    
    try:
        q_destino_valores = [float(x) for x in entrada.split()] #correção 
        q_final = np.array(q_destino_valores) #correção
        #cinematica:
        xyz, T_final = cinematica_nova(q_final, L1, L2, L3) #carrega a matriz
        print("-" * 30)
        print(f" Coordenadas Finais Calculadas (Manual):")
        print(f"  X={xyz[0]:.3f}, Y={xyz[1]:.3f}, Z={xyz[2]:.3f}")
        print("-" * 30)
      
      #trajetoria
        t_total, traj_posicoes, qd, qdd = traj_quintica(q_atual, q_final, time_s, dt)
        
        # 4. Simula o Movimento
        robo_DH.plot(traj_posicoes, backend="pyplot", dt=dt)
        
    except ValueError:
        print(" Erro: Números inválidos (use ponto).")
    except Exception as e:
        print(f" Ocorreu um erro: {e}")

plt.ioff()
plt.show()



        
        
