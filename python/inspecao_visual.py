"""
inspecao_visual.py - Inspeção visual por YOLO (EXTRA: +3 pts).

Sistema de inspeção visual do túnel usando YOLO (Ultralytics YOLOv8) para detectar
"objetos no teto" durante a inspeção, conforme o ponto extra do enunciado.

Encaixe na arquitetura: é mais um processo Python que conversa pelo MQTT, como o
simulador e as GUIs — NÃO altera o núcleo C++. Fluxo:
  1. ASSINA  robo/telemetria   -> observa o estado do robô.
  2. Quando o robô detecta uma anomalia, o LIDAR aciona a câmera (liga_camera passa
     a true). Na BORDA DE SUBIDA desse sinal, este serviço dispara uma inspeção.
  3. Roda o YOLO na próxima imagem da câmera (pasta inspecao_imagens/), obtendo os
     objetos detectados (classe + confiança).
  4. PUBLICA o resultado em robo/inspecao_visual (lido pela Simulação Visual) e
     salva a imagem anotada em inspecao_resultados/.

Observação: por não haver uma câmera real, as "capturas" vêm de uma pasta de imagens
(inspecao_imagens/). O serviço usa imagens de exemplo (baixadas na primeira execução)
se a pasta estiver vazia; basta substituí-las pelas imagens desejadas. O modelo é o
YOLOv8n pré-treinado (COCO, 80 classes), baixado automaticamente na 1ª execução.

Requer: pip install ultralytics --break-system-packages
Rodar com: python3 inspecao_visual.py
"""

import os
import time
import json
import glob
import urllib.request
import paho.mqtt.client as mqtt

# --- Configuração MQTT (deve bater com contrato_api.md) ---
BROKER = "localhost"
PORTA = 1883
TOPICO_TELEMETRIA = "robo/telemetria"        # entrada: estado do robô (gatilho)
TOPICO_INSPECAO = "robo/inspecao_visual"     # saída: resultado do YOLO

# --- Caminhos (relativos à pasta deste script) ---
RAIZ = os.path.dirname(os.path.abspath(__file__))
PASTA_IMAGENS = os.path.join(RAIZ, "inspecao_imagens")     # "capturas" da câmera
PASTA_RESULTADOS = os.path.join(RAIZ, "inspecao_resultados")  # imagens anotadas
MODELO = "yolov8n.pt"   # modelo nano pré-treinado (auto-download)
CONF_MIN = 0.30         # confiança mínima para reportar uma detecção

# Imagens de exemplo (baixadas se a pasta estiver vazia) — contêm objetos que o
# YOLO detecta de forma confiável, para demonstrar o pipeline de ponta a ponta.
IMAGENS_EXEMPLO = {
    "exemplo_1.jpg": "https://ultralytics.com/images/bus.jpg",
    "exemplo_2.jpg": "https://ultralytics.com/images/zidane.jpg",
}


def preparar_imagens():
    """Garante que exista ao menos uma imagem na pasta de capturas."""
    os.makedirs(PASTA_IMAGENS, exist_ok=True)
    os.makedirs(PASTA_RESULTADOS, exist_ok=True)

    existentes = glob.glob(os.path.join(PASTA_IMAGENS, "*.jpg")) + \
                 glob.glob(os.path.join(PASTA_IMAGENS, "*.png"))
    if existentes:
        return sorted(existentes)

    print("[YOLO] Pasta de imagens vazia; baixando exemplos para demonstração...")
    for nome, url in IMAGENS_EXEMPLO.items():
        destino = os.path.join(PASTA_IMAGENS, nome)
        try:
            urllib.request.urlretrieve(url, destino)
            print(f"[YOLO]   baixado: {nome}")
        except Exception as e:
            print(f"[YOLO]   falha ao baixar {nome}: {e}")

    return sorted(glob.glob(os.path.join(PASTA_IMAGENS, "*.jpg")))


class InspecaoVisual:
    def __init__(self):
        # Estado do gatilho (borda de subida de liga_camera)
        self.camera_anterior = False
        self.disparar = False
        self.x_disparo = 0.0

        # Imagens da câmera (round-robin a cada inspeção)
        self.imagens = preparar_imagens()
        self.idx_imagem = 0
        if not self.imagens:
            print("[YOLO] AVISO: nenhuma imagem disponível em inspecao_imagens/.")

        # Carrega o modelo YOLO (uma vez)
        print(f"[YOLO] Carregando modelo {MODELO} (1ª vez baixa ~6 MB)...")
        from ultralytics import YOLO
        self.model = YOLO(MODELO)
        print("[YOLO] Modelo pronto.")

        self._iniciar_mqtt()

    # ------------------------------------------------------------------ #
    #  MQTT
    # ------------------------------------------------------------------ #
    def _iniciar_mqtt(self):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="inspecao_visual")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        try:
            self.client.connect(BROKER, PORTA, keepalive=60)
            self.client.loop_start()  # rede em thread separada
        except Exception as e:
            print(f"[YOLO] Falha ao conectar no broker: {e}")
            print("[YOLO] Verifique se o mosquitto está rodando.")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe(TOPICO_TELEMETRIA)
        print(f"[YOLO] Conectado ao broker. Assinando {TOPICO_TELEMETRIA}")

    def _on_message(self, client, userdata, msg):
        """Só observa o gatilho; o YOLO (pesado) roda no laço principal."""
        try:
            t = json.loads(msg.payload.decode())
        except Exception:
            return
        camera = bool(t.get("liga_camera", False))
        # Borda de subida: a câmera acabou de ser acionada em uma anomalia.
        if camera and not self.camera_anterior:
            self.disparar = True
            self.x_disparo = float(t.get("x", 0.0))
        self.camera_anterior = camera

    # ------------------------------------------------------------------ #
    #  Inspeção (YOLO)
    # ------------------------------------------------------------------ #
    def _proxima_imagem(self):
        if not self.imagens:
            return None
        img = self.imagens[self.idx_imagem % len(self.imagens)]
        self.idx_imagem += 1
        return img

    def _inspecionar(self):
        img = self._proxima_imagem()
        if img is None:
            return

        # Roda o YOLO na "captura" da câmera.
        resultados = self.model(img, conf=CONF_MIN, verbose=False)
        r = resultados[0]

        objetos = []
        for box in r.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            objetos.append({"classe": self.model.names[cls], "conf": round(conf, 2)})

        # Salva a imagem anotada (caixas + rótulos) para o relatório/vídeo.
        nome_saida = os.path.join(PASTA_RESULTADOS,
                                  f"insp_x{self.x_disparo:06.1f}.jpg")
        try:
            r.save(nome_saida)
        except Exception as e:
            print(f"[YOLO] Falha ao salvar imagem anotada: {e}")
            nome_saida = ""

        # Publica o resultado para a Simulação Visual exibir.
        payload = {
            "x": round(self.x_disparo, 1),
            "n": len(objetos),
            "objetos": objetos,
            "imagem": os.path.basename(nome_saida) if nome_saida else "",
        }
        self.client.publish(TOPICO_INSPECAO, json.dumps(payload))

        # Log no terminal.
        if objetos:
            resumo = ", ".join(f"{o['classe']}({o['conf']:.2f})" for o in objetos)
        else:
            resumo = "nenhum objeto"
        print(f"[YOLO] Inspeção em x={self.x_disparo:.1f} m -> {resumo}")

    # ------------------------------------------------------------------ #
    #  Laço principal
    # ------------------------------------------------------------------ #
    def executar(self):
        print("[YOLO] Inspeção visual no ar. Aguardando acionamentos da câmera. "
              "Ctrl+C para encerrar.")
        try:
            while True:
                if self.disparar:
                    self.disparar = False
                    self._inspecionar()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[YOLO] Encerrando...")
        finally:
            self.client.loop_stop()
            self.client.disconnect()


def main():
    InspecaoVisual().executar()


if __name__ == "__main__":
    main()