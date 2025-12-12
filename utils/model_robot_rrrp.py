import roboticstoolbox as rtb
import numpy as np
import os

#                        DH table
#           +--------+---------+------+-------+
#           | theta  |    d    |  a   | alpha |
#           +--------+---------+------+-------+
#  Junta 1  | theta1 |    l1   |  0   |  pi/2 |
#  Junta 2  | theta2 |    0    |  l2  |   0   |
#  Junta 3  | theta3 |    0    |   0  |  pi/2 |
#  Junta 4  |   0    | l3 + d4 |   0  | pi/2  |
#           +--------+---------+------+-------+      

def robot_DH():
 # Comprimentos
 l1, l2, l3 = 1.0, 0.8, 0.3
#massas
 m1, m2, m3, m4 = 157.953, 42.761, 62.333, 4
 r1 = np.array([l1/2, 0, 0]) 
 r2 = np.array([l2/2, 0, 0])
 r3 = np.array([0, 0, l3/2])
 r4 = np.array([0, 0, 0.5/2])

 radius = 0.250 #raios dos links
#inercias
 I1 = [0.5 * m1 * radius**2, 
 (1/12) * m1 * (3*radius**2 + l1**2), 
 (1/12) * m1 * (3*radius**2 + l1**2)]

 I2 = [0.5 * m2 * radius**2, 
 (1/12) * m2 * (3*radius**2 + l2**2), 
 (1/12) * m2 * (3*radius**2 + l2**2)]

 I3 = [0.5 * m3 * radius**2, 
 (1/12) * m3 * (3*radius**2 + l3**2), 
 (1/12) * m3 * (3*radius**2 + l3**2)]

    #assumindo minima dimensao
 l4_len = 0.1 
 I4 = [0.5 * m4 * radius**2, 
 (1/12) * m4 * (3*radius**2 + l4_len**2), 
 (1/12) * m4 * (3*radius**2 + l4_len**2)]

 # Criação dos links (R-R-R-P)
 link1 = rtb.RevoluteDH(d=l1, a=0, alpha=np.pi/2,m=m1,r=r1,I=I1)
 link2 = rtb.RevoluteDH(d=0,  a=l2, alpha=0,m=m2,r=r2,I=I2)
 link3 = rtb.RevoluteDH(d=0,  a=0, alpha=np.pi/2,m=m3,r=r3,I=I3)
 link4 = rtb.PrismaticDH(theta=0, a=0, alpha=np.pi/2, offset=l3, qlim=[0, 1.0], m=m4,r=r4,I=I4)


 # Criação do robô 
 robot = rtb.DHRobot([link1, link2, link3, link4], name="Robo_Grupo4")

 return robot, l1, l2, l3

def robot_URDF():
 tld = os.path.expanduser('robot_quadramech_description')
 filepath = "urdf/robot_quadramech.xacro" 

#URDF_read finalmente funcionou
 links, name, urdf_string, urdf_filepath = rtb.robot.ERobot.URDF_read(filepath, tld=tld)

#criando robo
 robot_URDF = rtb.robot.ERobot(
    links,
    name=name,
    urdf_string=urdf_string,
    urdf_filepath=urdf_filepath
)

 return robot_URDF
