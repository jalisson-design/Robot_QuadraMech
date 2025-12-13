# Robot_QuadraMech
Este repositório contém a implementação completa das cinemáticas, execução de trajetória e análise dinâmica de um manipulador robótico de 4 Graus de Liberdade (DOF), configurado como R-R-R-P.  O projeto atende aos requisitos da disciplina de Dinâmica de sistema Robóticos e utiliza a biblioteca Robotics Toolbox for Python para simulação e validação.

### Requisitos Implementados:

Definição do Robô com DH com Massa/Inércia: (sem inercia até o momento, esperando URDF)

```
utils/model_robot_rrrp.py 
```

Cinemática Direta Manual e Trajetória:

```
utils/toolbox1.py
```
  
## Pré-Requisitos

Para a execução, é necessário a biblioteca roboticstoolbox do Peter Corke, para isso o arquivo dinamica_env.yaml já contem todas as dependências necessárias:

```
conda env create -f dinamica_env.yaml
```

### Ativando o Ambiente: 

```
conda activate dinamica 
```

### Execução do Simulador (Cinemática direta e Trajetória)
O script principal (cinematic_dir.py) executa um loop interativo no terminal para testar a cinemática e a execução de trajetória, cumprindo o requisito de interação.
 
 * Execute o script:
   ```
   python cinematic_dir.py
   ```

 *  O programa solicitará as posições finais das juntas (q1, q2, q3 em radianos e q4 em metros).
 *  O Console ira exibir as coordenadas (X,Y,Z) calculadas pela função ```cinematica_nova``` 

### Execução do Simulador (Cinemática indireta e Trajetória)
O script principal (cinematic_inv.py) executa um loop interativo no terminal para testar a cinemática e a execução de trajetória, cumprindo o requisito de interação.
 
 * Execute o script:
   ```
   python cinematic_inv.py
   ```

 *  O programa solicitará a posição final do atuador (x,y,z).
 *  O Console ira exibir as configurações de cada junta (q1, q2, q3, q4) calculadas pela função ```cinematica_inversa``` 

### Execução do Simulador (Análise Dinâmica e Trajetória)
O script principal (analise_din.py) executa uma movimentação referente a rotina do robô com sensor ultrassonico.
 
 * Execute o script:
   ```
   python analise_din.py
   ```

 *  O programa exibirá uma figura com as posições, velocidades e acelerações em cada junta, logo após exibirá o torque em cada junta e por última a simulação do movimento.

 
