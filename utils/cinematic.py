import numpy as np


def dh_matrix(theta, d, a, alpha):
   
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def calcular_fk_manual(q, tabela_dh):
    
    
    T_total = np.eye(4)
    
    
    for i, params in enumerate(tabela_dh):
        theta_offset, d, a, alpha = params
        
        
        theta_atual = q[i] + theta_offset
        
        
        Ti = dh_matrix(theta_atual, d, a, alpha)
        
        
        T_total = np.dot(T_total, Ti) 
        
    return T_total

def ik_jacobian_transpose(robot, target_pos, q_initial, tabela_dh, max_iter=200, tolerance=0.05, alpha=0.1):
    q_current = np.copy(q_initial)
    target_pos = np.array(target_pos)
    
    # Pega o primeiro efetuador disponível (provavelmente um dedo)
    link_ponta = robot.ee_links[0]
    
    erro_final = 100.0
    
    for i in range(max_iter):
        # 1. FK Manual (Usa apenas as 7 juntas do braço para calcular posição)
        T_current = calcular_fk_manual(q_current[:7], tabela_dh)
        current_pos = T_current[:3, 3] # Extrai X Y Z
        
        # 2. Erro
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)
        erro_final = error_norm
        
        if error_norm < tolerance:
            print(f"   -> Convergiu na iteração {i}. Erro: {error_norm:.4f}m")
            return q_current, True
        
        # 3. Jacobiano
        # O Toolbox calcula J para a cadeia cinemática até a ponta
        J = robot.jacob0(q_current, end=link_ponta)
        J_pos = J[:3, :] # Apenas translação (X, Y, Z)
        
        # 4. Atualização (Jacobian Transpose)
        # delta_q terá o tamanho do número de colunas de J (pode ser 7, 8 ou 9)
        delta_q = alpha * np.dot(J_pos.T, error)
        
        # --- CORREÇÃO DE TAMANHO (O PULO DO GATO) ---
        # Cria um vetor de atualização com zeros do tamanho TOTAL do robô (9)
        delta_q_full = np.zeros_like(q_current)
        
        # Preenche com os valores calculados até onde der
        # Se delta_q for tamanho 8, preenche indices 0 a 7. O indice 8 fica zero.
        n_cols = delta_q.shape[0]
        delta_q_full[:n_cols] = delta_q
        
        # Agora a soma é segura: (9,) + (9,)
        q_current += delta_q_full
        
    print(f"   -> AVISO: Não convergiu. Erro final: {erro_final:.4f}m")
    return q_current, False