import numpy as np

def cinematica_direta_manual(q, l1, l2, l3):
    """
    Calcula a posição x, y, z do efetuador manualmente.
    q: vetor com as posições das juntas [theta1, theta2, theta3, d4]
    """
    # Desempacota as variáveis das juntas
    th1, th2, th3, d4_var = q
    
    # Parâmetros DH (theta, d, a, alpha) para cada elo
    # Elo 1: Rotacional
    PT1 = [th1, l1, 0, np.pi/2]
    
    # Elo 2: Rotacional
    PT2 = [th2, 0, l2, 0]
    
    # Elo 3: Rotacional
    PT3 = [th3, 0, 0, np.pi/2]
    
    # Elo 4: Prismática (theta fixo em 0, d variável)
    # O offset l3 entra aqui no 'd'
    PT4 = [0, d4_var + l3, 0, np.pi/2] 
    
    # Monta as matrizes individuais
    A1 = matriz_transformacao(*PT1)
    A2 = matriz_transformacao(*PT2)
    A3 = matriz_transformacao(*PT3)
    A4 = matriz_transformacao(*PT4)
    
    # Multiplica em ordem: Base -> T1 -> T2 -> T3 -> T4 -> Efetuador
    T_final = A1 @ A2 @ A3 @ A4
    
    # A posição x, y, z está na última coluna da matriz (índices 0, 1, 2 da coluna 3)
    x = T_final[0, 3]
    y = T_final[1, 3]
    z = T_final[2, 3]
    
    return np.array([x, y, z]), T_final

def matriz_transformacao(theta, d, a, alpha):
    """
    Função auxiliar que cria a matriz 4x4 de um elo DH padrão
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1]

    ])

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
