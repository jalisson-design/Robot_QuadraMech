import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from utils.toolbox1 import cinematica_nova
from utils.model_robot_rrrp import robot_DH
from utils.toolbox1 import traj_quintica
from utils.toolbox1 import cinematica_inversa
import matplotlib as plot

#carrega o robo1.57
robo_DH, L1, L2, L3 = robot_DH()

# posição da junta inicial e final
q_atual = np.array([0.0, 0.0, 0.0, 0.0])
# q_final = np.array([np.pi/6, -np.pi/6, np.pi/4, 0.2])

xyz_atual, T_final = cinematica_nova(q_atual, L1, L2, L3)

# Tempo de simulacao
time_s = 5.0
dt = 0.05
steps = int(time_s / dt)
t = np.linspace(0, time_s, steps)


while True:
    print("-" * 30)
    print(f" Posição Atual Cartesiana: {xyz_atual}")
    print("Digite a posição final para o atuador")
    print("IMPORTANTE: O sensor no fim da ferramente deve estar perpendicular ao chão")
    print("Exemplo: 0.5 1 0.5")
    entrada = input(">>> Destino (ou 'sair'): ")
    
    if entrada == "sair":
      break
    
    try:
        q_destino_valores = [float(x) for x in entrada.split()] #correção 
        valores_final = np.array(q_destino_valores) #correção1.57
        f = valores_final
        q_out = cinematica_inversa(valores_final, L1, L2, L3)
        q1,q2,q3,d4 = q_out
        q = np.array([q1,q2,q3,d4])
        print(q)
        # print(" Posições das juntas (Manual):")
        # print(q_out)
        # print("-" * 30)
        #validacao
        xyz_a, T = cinematica_nova(q_out,L1,L2,L3)
        print(xyz_a)
        if np.allclose(xyz_a, valores_final, atol=1e-3):
         print("A cinemática inversa foi um sucesso!")
        else: 
         print("A cinemática inversa falhou: ")
         print(f"{xyz_a}é diferente de {valores_final}")

    except ValueError:
        print(" Erro: Números inválidos (use ponto).")
    except Exception as e:
        print(f" Ocorreu um erro: {e}")

plt.ioff()
plt.show()



        
        
