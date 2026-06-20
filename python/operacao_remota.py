"""
operacao_remota.py - Interface de Operação Remota do robô de inspeção (Tabela 2).

Painel de controle do operador humano. Faz duas coisas, conforme contrato_api.md:
  1. ASSINA  robo/telemetria -> exibe o estado do robô em tempo real
     (posição, teto, velocidade, confiança, modo, inspeção, câmera).
  2. PUBLICA robo/comandos   -> envia os comandos do operador
     (modo auto/manual, direção, setpoint de velocidade, limiar de anomalia).

Detalhe importante de threading: o Tkinter NÃO é thread-safe. O cliente MQTT roda
em uma thread própria (loop_start), então o callback de mensagem apenas GUARDA a
última telemetria; quem atualiza os widgets é um timer do Tkinter (root.after),
que roda na thread da interface. Isso evita corrupção da GUI.

Requer paho-mqtt 2.x e Tkinter (python3-tk). Rodar com: python3 operacao_remota.py
"""

import json
import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt

# --- Configuração MQTT (deve bater com contrato_api.md) ---
BROKER = "localhost"
PORTA = 1883
TOPICO_TELEMETRIA = "robo/telemetria"   # entrada: estado do robô
TOPICO_COMANDOS = "robo/comandos"       # saída: comandos do operador


class OperacaoRemota:
    def __init__(self, root):
        self.root = root
        self.root.title("Operação Remota — Robô de Inspeção de Túneis")
        self.root.geometry("440x600")
        self.root.resizable(False, False)

        # Última telemetria recebida (escrita pelo callback MQTT, lida pela GUI).
        self.ultima_telemetria = None
        self.conectado = False

        self._montar_interface()
        self._iniciar_mqtt()

        # Timer da GUI: atualiza os mostradores a cada 100ms (na thread do Tkinter).
        self.root.after(100, self._atualizar_tela)
        # Encerramento limpo ao fechar a janela.
        self.root.protocol("WM_DELETE_WINDOW", self._ao_fechar)

    # ------------------------------------------------------------------ #
    #  Construção da interface
    # ------------------------------------------------------------------ #
    def _montar_interface(self):
        pad = {"padx": 10, "pady": 6}

        # --- Painel de Telemetria (estado do robô) ---
        quadro_tele = ttk.LabelFrame(self.root, text="Telemetria (estado do robô)")
        quadro_tele.pack(fill="x", **pad)

        self.lbl = {}  # guarda os labels de valor para atualizar depois
        campos = [
            ("posicao", "Posição X:", "— m"),
            ("teto", "Teto (Y):", "— cm"),
            ("velocidade", "Velocidade:", "— m/s"),
            ("inclinacao", "Inclinação:", "— °"),
            ("confianca", "Confiança:", "— %"),
            ("modo", "Modo:", "—"),
            ("inspecao", "Inspeção:", "—"),
            ("camera", "Câmera:", "—"),
        ]
        for i, (chave, texto, inicial) in enumerate(campos):
            ttk.Label(quadro_tele, text=texto, width=14, anchor="e").grid(
                row=i, column=0, sticky="e", padx=6, pady=3)
            valor = ttk.Label(quadro_tele, text=inicial, width=18, anchor="w",
                              font=("TkDefaultFont", 10, "bold"))
            valor.grid(row=i, column=1, sticky="w", padx=6, pady=3)
            self.lbl[chave] = valor

        # --- Modo de operação ---
        quadro_modo = ttk.LabelFrame(self.root, text="Modo de operação")
        quadro_modo.pack(fill="x", **pad)
        ttk.Button(quadro_modo, text="Automático",
                   command=lambda: self._enviar("set_modo", "auto")).pack(
                       side="left", expand=True, fill="x", padx=8, pady=8)
        ttk.Button(quadro_modo, text="Manual",
                   command=lambda: self._enviar("set_modo", "manual")).pack(
                       side="left", expand=True, fill="x", padx=8, pady=8)

        # --- Direção (modo manual) ---
        quadro_dir = ttk.LabelFrame(self.root, text="Direção (modo manual)")
        quadro_dir.pack(fill="x", **pad)
        ttk.Button(quadro_dir, text="← Esquerda",
                   command=lambda: self._enviar("direcao", "esquerda")).pack(
                       side="left", expand=True, fill="x", padx=6, pady=8)
        ttk.Button(quadro_dir, text="Parar",
                   command=lambda: self._enviar("direcao", "parar")).pack(
                       side="left", expand=True, fill="x", padx=6, pady=8)
        ttk.Button(quadro_dir, text="Direita →",
                   command=lambda: self._enviar("direcao", "direita")).pack(
                       side="left", expand=True, fill="x", padx=6, pady=8)

        # --- Setpoint de velocidade ---
        quadro_vel = ttk.LabelFrame(self.root, text="Setpoint de velocidade")
        quadro_vel.pack(fill="x", **pad)
        self.entrada_vel = ttk.Entry(quadro_vel, width=10)
        self.entrada_vel.insert(0, "5.0")
        self.entrada_vel.pack(side="left", padx=8, pady=8)
        ttk.Label(quadro_vel, text="m/s").pack(side="left")
        ttk.Button(quadro_vel, text="Enviar",
                   command=self._enviar_velocidade).pack(
                       side="right", padx=8, pady=8)

        # --- Limiar de anomalia ---
        quadro_lim = ttk.LabelFrame(self.root, text="Limiar de detecção de anomalia")
        quadro_lim.pack(fill="x", **pad)
        self.entrada_lim = ttk.Entry(quadro_lim, width=10)
        self.entrada_lim.insert(0, "10")
        self.entrada_lim.pack(side="left", padx=8, pady=8)
        ttk.Label(quadro_lim, text="cm").pack(side="left")
        ttk.Button(quadro_lim, text="Enviar",
                   command=self._enviar_limiar).pack(
                       side="right", padx=8, pady=8)

        # --- Status da conexão ---
        self.lbl_status = ttk.Label(self.root, text="● Broker: conectando...",
                                    foreground="orange")
        self.lbl_status.pack(pady=8)

    # ------------------------------------------------------------------ #
    #  MQTT
    # ------------------------------------------------------------------ #
    def _iniciar_mqtt(self):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="operacao_remota")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        try:
            self.client.connect(BROKER, PORTA, keepalive=60)
            self.client.loop_start()  # rede em thread separada
        except Exception as e:
            print(f"[OP] Falha ao conectar no broker: {e}")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        self.conectado = True
        client.subscribe(TOPICO_TELEMETRIA)
        print(f"[OP] Conectado ao broker. Assinando {TOPICO_TELEMETRIA}")

    def _on_message(self, client, userdata, msg):
        # Apenas guarda; a atualização da tela é feita pela thread do Tkinter.
        try:
            self.ultima_telemetria = json.loads(msg.payload.decode())
        except Exception as e:
            print(f"[OP] Telemetria ignorada (JSON invalido): {e}")

    def _enviar(self, comando, valor):
        """Publica um comando no formato {comando, valor} do contrato."""
        msg = json.dumps({"comando": comando, "valor": valor})
        self.client.publish(TOPICO_COMANDOS, msg)
        print(f"[OP] Enviado: {msg}")

    def _enviar_velocidade(self):
        try:
            v = float(self.entrada_vel.get().replace(",", "."))
            self._enviar("set_velocidade", v)
        except ValueError:
            print("[OP] Velocidade invalida.")

    def _enviar_limiar(self):
        try:
            limiar = int(float(self.entrada_lim.get().replace(",", ".")))
            self._enviar("set_limiar", limiar)
        except ValueError:
            print("[OP] Limiar invalido.")

    # ------------------------------------------------------------------ #
    #  Atualização da tela (thread do Tkinter)
    # ------------------------------------------------------------------ #
    def _atualizar_tela(self):
        # Status do broker
        if self.conectado:
            self.lbl_status.config(text="● Broker: conectado", foreground="green")
        else:
            self.lbl_status.config(text="● Broker: desconectado", foreground="red")

        # Atualiza os mostradores com a última telemetria recebida
        t = self.ultima_telemetria
        if t is not None:
            self.lbl["posicao"].config(text=f"{t.get('x', 0):.1f} m")
            self.lbl["teto"].config(text=f"{t.get('y', 0)} cm")
            self.lbl["velocidade"].config(text=f"{t.get('velocidade', 0):.1f} m/s")

            # Inclinação (IMU): seta indica subida/descida/plano
            inc = t.get("inclinacao", 0.0)
            seta = "↗" if inc > 0.5 else ("↘" if inc < -0.5 else "→")
            self.lbl["inclinacao"].config(text=f"{inc:+.1f}° {seta}")

            self.lbl["confianca"].config(text=f"{t.get('confianca', 0)} %")

            modo = t.get("modo", "—")
            self.lbl["modo"].config(text=modo.upper(),
                                    foreground="blue" if modo == "auto" else "purple")

            inspecao = t.get("inspecao", False)
            self.lbl["inspecao"].config(
                text="ATIVA" if inspecao else "não",
                foreground="red" if inspecao else "black")

            camera = t.get("liga_camera", False)
            self.lbl["camera"].config(
                text="LIGADA" if camera else "desligada",
                foreground="red" if camera else "black")

        # Reagenda a próxima atualização
        self.root.after(100, self._atualizar_tela)

    def _ao_fechar(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    OperacaoRemota(root)
    root.mainloop()


if __name__ == "__main__":
    main()