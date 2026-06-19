# ATR-Trabalho-Final-2026-1

Trabalho final da disciplina de Automação em Tempo Real (2026/1) — sistema
embarcado em C++ e simulação para robô autônomo de inspeção de túneis.

O robô percorre um túnel mapeando o perfil do teto para detectar anomalias
(buracos e saliências). O sistema é dividido em três processos que se comunicam
via MQTT:

- **Robô embarcado (C++):** núcleo de tempo real — navegação (PID), odometria,
  reconstrução do teto (LIDAR), inspeção por câmera e coleta de dados.
- **Simulador físico (Python):** simula a mecânica newtoniana e os sensores,
  fechando o laço com os atuadores do robô.
- **Operação Remota (Python):** interface gráfica do operador, com comandos e
  telemetria em tempo real.

A especificação dos tópicos e mensagens MQTT está em
[`contrato_api.md`](contrato_api.md).

## Estrutura do repositório

```
include/        Cabeçalhos C++ (BufferCompartilhado.hpp)
src/            Código-fonte C++ do robô embarcado
python/         Simulador e Operação Remota (Python)   [Etapa 2]
CMakeLists.txt  Build do robô embarcado
contrato_api.md Contrato da API MQTT
requirements.txt Dependências Python
```

## Dependências

Para rodar o sistema completo (robô + simulação + operação remota) em uma
máquina, instale **todos** os itens abaixo. Os dois integrantes mantêm o
ambiente completo para desenvolver e testar de forma independente.

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

### Stack Python (simulação + operação remota)

```bash
sudo apt install -y python3 python3-pip python3-tk
pip install -r requirements.txt
```

## Como compilar (robô embarcado)

```bash
cmake -B build
cmake --build build
```

Gera dois executáveis em `build/`: `robo_embarcado` (sistema principal) e
`testes` (teste unitário do buffer).

## Como executar

### Modo offline (sem simulador externo)

Usa o mock interno de sensores. Bom para testar o núcleo C++ isoladamente.

```bash
./build/robo_embarcado --offline
```

### Modo online (laço fechado com a simulação)

Requer o broker MQTT e os scripts Python em execução. Ordem sugerida:

```bash
# 1. Garantir que o broker está ativo
sudo systemctl start mosquitto      # ou: mosquitto -v

# 2. Iniciar o simulador físico (Python)
python3 python/simulador.py

# 3. Iniciar a Operação Remota (Python)
python3 python/operacao_remota.py

# 4. Iniciar o robô embarcado em modo online
./build/robo_embarcado --online
```

Encerrar o robô no modo online: `Ctrl+C`.

> Um script único de inicialização (que sobe broker, robô e scripts Python de
> uma vez) será adicionado na fase de integração.

## Saída

O robô gera o arquivo `log_inspecao.csv` no diretório de execução, com colunas:
`Timestamp_ms, Posicao_X_m, Posicao_Y_cm, Confianca_%`.

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