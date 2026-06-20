"""
simulador.py - Simulador físico do robô de inspeção (LAÇO FECHADO).

Substitui o mock.cpp da Etapa 1. Em vez de injetar valores fixos, este script:
  1. ASSINA  robo/atuadores  -> recebe a aceleração comandada pelo robô (C++).
  2. Aplica as LEIS DE NEWTON para mover o robô (a -> v -> x), permitindo avanço
     E recuo (velocidade negativa).
  3. Gera os SENSORES e PUBLICA em robo/sensores:
       - encoder: troca de estado a cada metro percorrido.
       - sentido: sinal de direção do movimento (+1 avanço, -1 recuo, 0 parado),
         análogo ao que um encoder em quadratura forneceria. Permite à odometria
         contar a distância com o sinal correto (avanço soma, recuo subtrai).
       - lidar:   altura do teto no ponto atual (perfil do túnel) + ruído de medição.
       - imu_pitch: inclinação (pitch) do piso no ponto atual, em graus, medida por
         uma IMU simulada (EXTRA: túnel com declive). A física aplica a componente
         da gravidade na rampa, e o controle do robô usa essa leitura para compensar
         a subida/descida via feed-forward.

Assim o laço fecha de verdade:
   atuador (C++) -> física (aqui) -> sensores (aqui) -> robô (C++) -> atuador ...

Requer paho-mqtt 2.x. Rodar com: python3 simulador.py
"""

import json
import time
import math
import random
import paho.mqtt.client as mqtt

# --- Configuração MQTT (deve bater com contrato_api.md) ---
BROKER = "localhost"
PORTA = 1883
TOPICO_ATUADORES = "robo/atuadores"   # entrada: o que o robô comanda
TOPICO_SENSORES = "robo/sensores"     # saída: o que o robô "sente"

# --- Parâmetros da física ---
DT = 0.05          # passo de integração e de publicação (s)
ACEL_MAX = 4.0     # aceleração a 100% de atuação (m/s^2)
ARRASTO = 0.5      # arrasto viscoso (atrito proporcional à velocidade)
RUIDO_LIDAR = 3.0  # desvio-padrão do ruído de medição do lidar (cm)
LIMIAR_SENTIDO = 0.05  # velocidade mínima (m/s) para considerar o robô em movimento
GRAVIDADE = 9.81   # aceleração da gravidade (m/s^2), para a física na rampa (EXTRA: declive)
RUIDO_IMU = 0.2    # desvio-padrão do ruído de medição da IMU (graus)

# --- Perfil do túnel (a "verdade" física do teto) ---
TETO_BASE = 200    # altura nominal do teto (cm)

# Anomalias fixas ao longo do túnel: cada tupla é (x_inicio_m, x_fim_m, altura_cm).
# Altura MAIOR que a base = BURACO (teto mais alto); MENOR = SALIÊNCIA (mais baixo).
# Ficam bem distribuídas, com trechos retos (sem falhas) entre elas, como na Figura 2,
# e variam em largura (2 a 4 m) e profundidade para tornar a inspeção mais rica.
ANOMALIAS = [
    (  8.0,  10.0, 280),   # buraco
    ( 15.0,  17.0, 150),   # saliência
    ( 24.0,  27.0, 285),   # buraco largo
    ( 34.0,  36.0, 165),   # saliência
    ( 44.0,  46.0, 255),   # buraco raso
    ( 54.0,  58.0, 140),   # saliência larga e funda
    ( 66.0,  68.0, 275),   # buraco
    ( 76.0,  79.0, 170),   # saliência
    ( 88.0,  91.0, 282),   # buraco fundo
    ( 99.0, 101.0, 248),   # buraco raso
    (108.0, 112.0, 155),   # saliência larga
    (120.0, 122.0, 270),   # buraco
]


def perfil_teto(x):
    """Altura REAL do teto (cm) na posição x (m).
    Retorna a altura da anomalia se x cair em um trecho de falha; caso contrário,
    o teto nominal (trecho reto, sem falhas). Buraco = teto mais alto (valor maior);
    saliência = teto mais baixo (valor menor)."""
    for inicio, fim, altura in ANOMALIAS:
        if inicio <= x <= fim:
            return altura
    return TETO_BASE


# Inclinações fixas do piso ao longo do túnel: (x_inicio_m, x_fim_m, angulo_graus).
# Ângulo > 0 = SUBIDA (declive positivo); < 0 = DESCIDA. Fora dos trechos, piso plano.
# Mantidos suaves (até ~6°) para que o robô consiga vencer a rampa com o motor
# (em ~6° a gravidade tira ~1,0 m/s² de uma aceleração máxima de 4,0 m/s²).
INCLINACOES = [
    ( 20.0,  32.0,  5.0),   # subida
    ( 48.0,  60.0, -5.0),   # descida
    ( 78.0,  92.0,  6.0),   # subida mais íngreme
    (104.0, 118.0, -4.0),   # descida suave
]


def inclinacao(x):
    """Inclinação REAL do piso (graus) na posição x (m). 0 = plano.
    Positivo = subida no sentido +X; negativo = descida."""
    for inicio, fim, angulo in INCLINACOES:
        if inicio <= x <= fim:
            return angulo
    return 0.0


class Simulador:
    """Mantém o estado físico do robô e avança a simulação passo a passo."""

    def __init__(self):
        self.x = 0.0              # posição (m)
        self.v = 0.0              # velocidade (m/s) — pode ser negativa (recuo)
        self.aceleracao_pct = 0   # último atuador recebido (-100 a 100 %)
        self.encoder = False      # estado atual do encoder
        self.sentido = 0          # +1 avanço, -1 recuo, 0 parado
        self.ultimo_metro = 0     # último metro inteiro já contabilizado
        self.lidar = TETO_BASE    # última leitura do lidar (cm)
        self.imu_pitch = 0.0      # inclinação medida pela IMU (graus)

    def set_atuador(self, pct):
        """Recebe a aceleração comandada pelo robô, saturada em [-100, 100]."""
        self.aceleracao_pct = max(-100, min(100, pct))

    def passo(self, dt):
        """Avança a física um passo dt e atualiza os sensores."""
        # Inclinação do trecho atual (graus -> rad) para a física da rampa.
        theta_graus = inclinacao(self.x)
        theta_rad = math.radians(theta_graus)

        # Lei de Newton na rampa: atuação - arrasto - componente da gravidade.
        # Em subida (theta>0) a gravidade DESACELERA; em descida (theta<0) ACELERA.
        a = ((self.aceleracao_pct / 100.0) * ACEL_MAX
             - ARRASTO * self.v
             - GRAVIDADE * math.sin(theta_rad))
        self.v += a * dt
        self.x += self.v * dt

        # Sentido do movimento (para a odometria contar metros com o sinal correto).
        if self.v > LIMIAR_SENTIDO:
            self.sentido = 1
        elif self.v < -LIMIAR_SENTIDO:
            self.sentido = -1
        else:
            self.sentido = 0

        # Encoder: troca de estado a CADA metro percorrido. Usa o cruzamento de um
        # inteiro de x (em qualquer sentido), então funciona para avanço e recuo.
        metro_atual = int(math.floor(self.x))
        if metro_atual != self.ultimo_metro:
            self.encoder = not self.encoder
            self.ultimo_metro = metro_atual

        # Lidar: altura real do teto + ruído gaussiano de medição.
        leitura = perfil_teto(self.x) + random.gauss(0, RUIDO_LIDAR)
        self.lidar = int(round(leitura))

        # IMU: mede o pitch (inclinação) do piso no ponto atual, com ruído.
        self.imu_pitch = theta_graus + random.gauss(0, RUIDO_IMU)

    def leitura_sensores(self):
        """Empacota o estado dos sensores no formato do contrato."""
        return {
            "encoder": self.encoder,
            "sentido": self.sentido,
            "lidar": self.lidar,
            "imu_pitch": round(self.imu_pitch, 2),
        }


# Instância única do simulador, compartilhada com os callbacks MQTT.
sim = Simulador()


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[SIM] Conectado ao broker (rc={reason_code}). Assinando {TOPICO_ATUADORES}")
    client.subscribe(TOPICO_ATUADORES)


def on_message(client, userdata, msg):
    """Recebe a aceleração comandada pelo robô e repassa ao simulador."""
    try:
        dados = json.loads(msg.payload.decode())
        if "aceleracao" in dados:
            sim.set_atuador(int(dados["aceleracao"]))
    except Exception as e:
        print(f"[SIM] Mensagem ignorada (JSON invalido): {e}")


def main():
    # paho-mqtt 2.x: o 1o argumento é a versão da API de callback.
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="simulador_fisico")
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(BROKER, PORTA, keepalive=60)
    except Exception as e:
        print(f"[SIM] Falha ao conectar no broker: {e}")
        print("[SIM] Verifique se o mosquitto esta rodando.")
        return

    client.loop_start()  # rede roda em thread separada; o loop de física fica aqui
    print("[SIM] Simulador iniciado. Ctrl+C para encerrar.")

    try:
        while True:
            sim.passo(DT)
            client.publish(TOPICO_SENSORES, json.dumps(sim.leitura_sensores()))
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\n[SIM] Encerrando...")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()