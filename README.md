# ATR-Trabalho-Final-2026-1

Trabalho final da disciplina de Automação em Tempo Real (2026/1) — sistema
embarcado em C++ e simulação para robô autônomo de inspeção de túneis.

O robô percorre um túnel mapeando o perfil do teto para detectar anomalias
(buracos e saliências). O sistema é dividido em processos que se comunicam via
MQTT:

- **Robô embarcado (C++):** núcleo de tempo real — navegação (PID), odometria,
  reconstrução do teto (LIDAR), inspeção por câmera e coleta de dados.
- **Simulador físico (Python):** simula a mecânica newtoniana e os sensores,
  fechando o laço com os atuadores do robô.
- **Operação Remota (Python):** interface gráfica do operador, com comandos e
  telemetria em tempo real.
- **Simulação visual (Python):** animação 2D do robô percorrendo o túnel e do
  perfil do teto reconstruído.
- **Inspeção visual por YOLO (Python, EXTRA):** quando a câmera é acionada em uma
  anomalia, roda YOLOv8 para detectar objetos na "captura" e publica o resultado.

A especificação dos tópicos e mensagens MQTT está em
[`contrato_api.md`](contrato_api.md).

### Funcionalidade extra: IMU e túnel com declive

O túnel possui trechos com inclinação (subidas e descidas). O simulador aplica a
componente da gravidade na rampa, e uma **IMU simulada** mede o pitch do robô. O
controle de navegação usa essa leitura para compensar a rampa por feed-forward
(mantendo a velocidade na subida/descida), a inclinação é exibida nas GUIs (com o
robô inclinando na visualização) e registrada no `log_inspecao.csv`.

### Funcionalidade extra: inspeção visual com YOLO

O processo `python/inspecao_visual.py` faz a inspeção visual com **YOLOv8**
(Ultralytics). Ele observa a telemetria e, na borda de subida de `liga_camera`
(câmera acionada em uma anomalia), roda o YOLO em uma imagem de "captura" da
câmera, detecta objetos (classe + confiança), publica o resultado em
`robo/inspecao_visual` (exibido na Simulação Visual) e salva a imagem anotada em
`python/inspecao_resultados/`.

As "capturas" ficam em `python/inspecao_imagens/`. Se a pasta estiver vazia na
primeira execução, o serviço baixa imagens de exemplo para demonstrar o pipeline;
basta substituí-las pelas imagens desejadas. O modelo `yolov8n.pt` (~6 MB) é
baixado automaticamente na primeira execução.

## Estrutura do repositório

```
include/          Cabeçalhos C++ (BufferCompartilhado.hpp, Profiler.hpp)
src/              Código-fonte C++ do robô embarcado
python/           Simulador, Operação Remota, Simulação Visual e Inspeção YOLO (Python)
CMakeLists.txt    Build do robô embarcado
iniciar.sh        Launcher único do sistema completo
contrato_api.md   Contrato da API MQTT
requirements.txt  Dependências Python
```

## Dependências

Para rodar o sistema completo em uma máquina, instale **todos** os itens abaixo.
Os dois integrantes mantêm o ambiente completo para desenvolver e testar de forma
independente.

> **Recomendação:** rodar tudo dentro do WSL (Ubuntu). Assim o broker, o C++ e o
> Python se comunicam por `localhost`, sem complicação de rede.

### Infraestrutura comum

```bash
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y mosquitto mosquitto-clients
```

### Stack C++ (robô embarcado)

```bash
sudo apt install -y libboost-all-dev
sudo apt install -y libpaho-mqtt-dev libpaho-mqttpp-dev
sudo apt install -y nlohmann-json3-dev
```

### Stack Python (simulação + operação remota + visualização + inspeção YOLO)

```bash
sudo apt install -y python3 python3-pip python3-tk
pip install -r requirements.txt --break-system-packages
```

> A flag `--break-system-packages` é necessária no Ubuntu 24.04 (Python 3.12),
> que bloqueia instalação global por padrão (PEP 668). Alternativa: usar um
> ambiente virtual (`python3 -m venv venv && source venv/bin/activate`).
>
> O `requirements.txt` inclui o `ultralytics` (YOLOv8) para a inspeção visual
> extra; ele puxa o `torch` (CPU) automaticamente. É um download grande na
> primeira instalação.

## Como compilar (robô embarcado)

```bash
cmake -B build
cmake --build build
```

Gera dois executáveis em `build/`: `robo_embarcado` (sistema principal) e
`testes` (teste unitário do buffer).

## Como executar

### Sistema completo (recomendado) — launcher único

Sobe broker, robô, simulador e as duas GUIs de uma vez. Encerra tudo com Ctrl+C:

```bash
./iniciar.sh
```

(na primeira vez, dê permissão de execução: `chmod +x iniciar.sh`)

### Execução manual (para depuração)

Cada processo em um terminal:

```bash
# 1. Broker
mosquitto                                   # ou: sudo systemctl start mosquitto

# 2. Simulador físico
python3 python/simulador.py

# 3. Robô embarcado (modo online)
./build/robo_embarcado --online

# 4. Operação Remota (GUI)
python3 python/operacao_remota.py

# 5. Simulação visual (GUI)
python3 python/visualizacao.py

# 6. Inspeção visual por YOLO (EXTRA)
python3 python/inspecao_visual.py
```

### Modo offline (núcleo C++ isolado, sem MQTT)

Usa o mock interno de sensores; bom para testar só o robô:

```bash
./build/robo_embarcado --offline
```

## Saída

O robô gera o arquivo `log_inspecao.csv` no diretório de execução, com colunas:
`Timestamp_ms, Posicao_X_m, Posicao_Y_cm, Confianca_%, Inclinacao_graus`.

Ao encerrar, o robô também imprime os WCET medidos das tarefas, base para a
análise de escalonabilidade (ver `escalonabilidade.md`).

## Testes

```bash
./build/testes
```

Valida operações do buffer thread-safe (push/pop, descarte por overflow,
fechamento seguro).

## Autores

- Bruna Faria Rodrigues
- Leonardo Martins Marques Oliveira

Universidade Federal de Minas Gerais — Engenharia de Controle e Automação.