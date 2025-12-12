import numpy as np

def traj_quintica(q_start, q_end, tempo_total, dt):
    """
    Baseado no exemplo05.py da monitoria.
    """
    steps = int(tempo_total / dt)
    t = np.linspace(0, tempo_total, steps)
    n_juntas = len(q_start)
    
    #inicializa matrizes
    q_traj = np.zeros((steps, n_juntas))
    qd_traj = np.zeros((steps, n_juntas))
    qdd_traj = np.zeros((steps, n_juntas))

    for i, t_i in enumerate(t):
        s = t_i / tempo_total
        
        #coeficientes do polinômio de jerk mínimo: 10, -15, 6
        #posição
        poly_pos = 10*s**3 - 15*s**4 + 6*s**5
        q_traj[i, :] = q_start + (q_end - q_start) * poly_pos
        
        #velocidade (Derivada 1)
        poly_vel = (30*s**2 - 60*s**3 + 30*s**4) / tempo_total
        qd_traj[i, :] = (q_end - q_start) * poly_vel
        
        #aceleração (Derivada 2)
        poly_acc = (60*s - 180*s**2 + 120*s**3) / (tempo_total**2)
        qdd_traj[i, :] = (q_end - q_start) * poly_acc
        
    return t, q_traj, qd_traj, qdd_traj

def cinematica_nova(q, l1, l2, l3): #utilizar essa:
    
    th1, th2, th3, d4_var = q
    C1 = np.cos(th1); S1 = np.sin(th1)
    C2 = np.cos(th2); S2 = np.sin(th2)
    C3 = np.cos(th3); S3 = np.sin(th3)

    C23 = C2 * C3 - S2 * S3 # cos(th2 + th3)
    S23 = S2 * C3 + C2 * S3 # sin(th2 + th3)
    D4 = l3 + d4_var

    T_final = np.array([
        [ C1*C23,S1*C23,-S23, 0 ],
        [ C1*S23,   S1*S23, C23, 0 ],
        [ -S1, C1, 0, 0 ],
        [ C1 * (l2 * C2 + D4 * S23),   S1 * (l2 * C2 + D4 * S23),   l1 + l2 * S2 - D4 * C23,   1 ]
    ]).T 


    # T14 = C1*(l2*C2 + D4*S23)+l3*S1
    # T24 = S1*(l2*C2+D4*S23)-l3*C1
    # T34 = C23, l1-D4*C23


    # T_final = np.array([ 
    #     [C1*C23, S1, C1*S23, T14],
    #     [S1*C23, -C1, S1*S23, T24],
    #     [S23, 0 , T34 ],
    #     [0,0,0,1]
    # ])

    x = T_final[0, 3]
    y = T_final[1, 3]
    z = T_final[2, 3]
    
    return np.array([x, y, z]), T_final

