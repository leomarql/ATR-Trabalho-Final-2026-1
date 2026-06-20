#!/usr/bin/env bash
#
# iniciar.sh - Launcher único do sistema (instrução geral 5 do enunciado).
#
# Sobe, em um único comando, todos os processos do sistema completo:
#   1. Broker MQTT (mosquitto) — se ainda não estiver rodando.
#   2. Robô embarcado (C++) em modo online.
#   3. Simulador físico (Python).
#   4. Operação Remota (GUI Python).
#   5. Simulação visual (GUI Python).
#
# Encerra TODOS os processos de uma vez ao pressionar Ctrl+C.
#
# Uso:  ./iniciar.sh
# (rode a partir da raiz do projeto; use referências relativas, conforme o enunciado)

set -u

# Caminhos relativos à raiz do projeto (onde este script está).
RAIZ="$(cd "$(dirname "$0")" && pwd)"
ROBO="$RAIZ/build/robo_embarcado"
PYDIR="$RAIZ/python"

# Lista de PIDs que este script iniciou, para encerrar no final.
PIDS=()
MOSQUITTO_INICIADO=0

# --- Encerramento limpo: mata tudo que foi iniciado aqui ---
encerrar() {
    echo ""
    echo "[LAUNCHER] Encerrando o sistema..."
    # Envia SIGINT ao robô primeiro (para o shutdown gracioso dele)
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    sleep 1
    # Garante o encerramento de quem sobrou
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    # Se este script subiu o mosquitto, derruba também
    if [ "$MOSQUITTO_INICIADO" -eq 1 ]; then
        kill "$MOSQUITTO_PID" 2>/dev/null
    fi
    echo "[LAUNCHER] Sistema encerrado."
    exit 0
}
trap encerrar INT TERM

# --- Verificações iniciais ---
if [ ! -x "$ROBO" ]; then
    echo "[LAUNCHER] ERRO: executável não encontrado em $ROBO"
    echo "[LAUNCHER] Compile primeiro: cmake -B build && cmake --build build"
    exit 1
fi

# --- 1. Broker MQTT ---
if pgrep -x mosquitto >/dev/null; then
    echo "[LAUNCHER] Broker mosquitto já está rodando."
else
    echo "[LAUNCHER] Iniciando broker mosquitto..."
    mosquitto >/dev/null 2>&1 &
    MOSQUITTO_PID=$!
    MOSQUITTO_INICIADO=1
    sleep 1
fi

# --- 2. Simulador físico (Python) ---
echo "[LAUNCHER] Iniciando simulador físico..."
python3 "$PYDIR/simulador.py" &
PIDS+=($!)
sleep 1

# --- 3. Robô embarcado (C++) em modo online ---
echo "[LAUNCHER] Iniciando robô embarcado (online)..."
"$ROBO" --online &
PIDS+=($!)
sleep 1

# --- 4. Operação Remota (GUI) ---
echo "[LAUNCHER] Iniciando Operação Remota..."
python3 "$PYDIR/operacao_remota.py" &
PIDS+=($!)

# --- 5. Simulação visual (GUI) ---
echo "[LAUNCHER] Iniciando Simulação Visual..."
python3 "$PYDIR/visualizacao.py" &
PIDS+=($!)

echo ""
echo "[LAUNCHER] Sistema completo no ar. Pressione Ctrl+C para encerrar tudo."

# Mantém o script vivo até o Ctrl+C
wait