import roboticstoolbox as rtb
import numpy as np

#                        DH table
#           +--------+---------+------+-------+
#           | theta  |    d    |  a   | alpha |
#           +--------+---------+------+-------+
#  Junta 1  | theta1 |    l1   |  0   |  pi/2 |
#  Junta 2  | theta2 |    0    |  l2  |   0   |
#  Junta 3  | theta3 |    0    |   0  |  pi/2 |
#  Junta 4  |   0    | l3 + d4 |   0  | pi/2  |
#           +--------+---------+------+-------+      

def robot():
 # Comprimentos
 l1, l2, l3 = 1.0, 0.8, 0.3

 # Criação dos links (R-R-P-R)
 link1 = rtb.RevoluteDH(d=l1, a=0, alpha=np.pi/2)
 link2 = rtb.RevoluteDH(d=0,  a=l2, alpha=0)
 link3 = rtb.RevoluteDH(d=0,  a=0, alpha=np.pi/2)
 link4 = rtb.PrismaticDH(theta=0, a=0, alpha=np.pi/2, offset=l3, qlim=[0, 1.0])


 # Criação do robô 
 robo = rtb.DHRobot([link1, link2, link3, link4], name="Robo_Grupo4")

 return robo, l1, l2, l3