"""
visualizacao.py - Simulação visual 2D do robô de inspeção (pygame).

Mostra, em vista lateral e em tempo real:
  - O robô (carrinho) percorrendo o túnel no eixo X.
  - O perfil do teto sendo RECONSTRUÍDO conforme as medições do lidar chegam
    (equivalente ao "gráfico de dados lidar processados" da Figura 2 do enunciado).
  - Destaque visual quando a inspeção/câmera é acionada em uma anomalia.
  - Opcionalmente, o teto "real" (ground truth) ao fundo, para comparar a
    realidade física com o que o robô mede (percepção).

Fonte de dados: assina robo/telemetria (posição x, teto y, modo, inspeção, etc.),
conforme contrato_api.md. É um processo independente — não precisa do simulador
no mesmo processo, só do broker MQTT no ar.

Requer paho-mqtt 2.x e pygame. Rodar com: python3 visualizacao.py
"""

import json
import sys
import math
import pygame
import paho.mqtt.client as mqtt

# Tenta importar o perfil real do túnel (teto e inclinação do piso) para desenhá-lo
# ao fundo. Se falhar (ex.: rodando fora da pasta), apenas não desenha o ground truth.
try:
    from simulador import perfil_teto, inclinacao, TETO_BASE
    TEM_GROUND_TRUTH = True
except Exception:
    TETO_BASE = 200
    TEM_GROUND_TRUTH = False
    def inclinacao(x):
        return 0.0

# --- Configuração MQTT ---
BROKER = "localhost"
PORTA = 1883
TOPICO_TELEMETRIA = "robo/telemetria"

# --- Configuração da janela ---
LARGURA, ALTURA = 960, 540
FPS = 60

# --- Mapeamento mundo -> tela ---
METROS_VISIVEIS = 26.0          # largura do túnel visível (m)
MARGEM = 40                     # margem lateral (px)
ESCALA_X = (LARGURA - 2 * MARGEM) / METROS_VISIVEIS  # px por metro
ESCALA_Y = 1.4                  # px por cm (vertical, para o teto)
ESCALA_PISO = ESCALA_X          # px por metro na vertical do piso: igual à horizontal,
                                # para que a rampa na tela tenha o ângulo real (e o robô,
                                # que tomba pelo pitch, encaixe exatamente no chão).
TETO_TELA_BASE = 130            # y de tela do teto nominal (px)
CHAO_TELA = 430                 # y de tela do chão sob o robô (px)

# --- Perfil de altura do piso (subida acumulada em metros, integrando a inclinação) ---
# A inclinação dá o ângulo do piso em cada x; a altura é a integral de tan(ângulo).
# Pré-computado uma vez (a inclinação é fixa), com interpolação linear na consulta.
_PISO_PASSO = 0.25
_PISO_XMAX = 200.0


def _construir_perfil_piso():
    alturas = [0.0]
    x, h = 0.0, 0.0
    while x < _PISO_XMAX:
        h += math.tan(math.radians(inclinacao(x))) * _PISO_PASSO
        x += _PISO_PASSO
        alturas.append(h)
    return alturas


_PISO_H = _construir_perfil_piso() if TEM_GROUND_TRUTH else [0.0]


def altura_piso(x):
    """Altura do piso (subida acumulada, em metros) na posição x (m). 0 = nível inicial."""
    if not TEM_GROUND_TRUTH or x <= 0:
        return 0.0
    i = int(x / _PISO_PASSO)
    if i >= len(_PISO_H) - 1:
        return _PISO_H[-1]
    frac = (x - i * _PISO_PASSO) / _PISO_PASSO
    return _PISO_H[i] + frac * (_PISO_H[i + 1] - _PISO_H[i])

# --- Cores ---
COR_FUNDO = (18, 18, 24)
COR_ROCHA = (70, 62, 55)
COR_ROCHA_ESC = (50, 44, 38)
COR_CHAO = (60, 55, 50)
COR_CHAO_LINHA = (95, 86, 76)   # linha de superfície do piso (rampa)
COR_PERFIL = (90, 200, 120)     # perfil medido (verde)
COR_REAL = (90, 90, 110)        # teto real ao fundo (cinza)
COR_ROBO = (80, 170, 230)
COR_ROBO_INSP = (235, 90, 80)   # robô durante inspeção (vermelho)
COR_TEXTO = (220, 220, 220)
COR_DESTAQUE = (235, 200, 80)


class Visualizacao:
    def __init__(self):
        self.telemetria = None       # última telemetria recebida (dict)
        self.perfil_medido = {}      # {metro(int): y_cm} reconstruído pelas medições
        self.conectado = False

        # --- pygame ---
        pygame.init()
        self.tela = pygame.display.set_mode((LARGURA, ALTURA))
        pygame.display.set_caption("Simulação Visual — Robô de Inspeção de Túneis")
        self.fonte = pygame.font.SysFont("Arial", 18)
        self.fonte_pq = pygame.font.SysFont("Arial", 14)
        self.relogio = pygame.time.Clock()

        self._iniciar_mqtt()

    # ------------------------------------------------------------------ #
    #  MQTT
    # ------------------------------------------------------------------ #
    def _iniciar_mqtt(self):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="visualizacao")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        try:
            self.client.connect(BROKER, PORTA, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            print(f"[VIS] Falha ao conectar no broker: {e}")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        self.conectado = True
        client.subscribe(TOPICO_TELEMETRIA)
        print(f"[VIS] Conectado ao broker. Assinando {TOPICO_TELEMETRIA}")

    def _on_message(self, client, userdata, msg):
        try:
            t = json.loads(msg.payload.decode())
            self.telemetria = t
            # Reconstrói o perfil do teto: guarda a leitura y para cada metro x.
            x = t.get("x", 0.0)
            y = t.get("y", TETO_BASE)
            self.perfil_medido[round(x)] = y
        except Exception as e:
            print(f"[VIS] Telemetria ignorada: {e}")

    # ------------------------------------------------------------------ #
    #  Conversões mundo -> tela (com câmera que segue o robô)
    # ------------------------------------------------------------------ #
    def _camera_x(self):
        """Posição (em metros) da borda esquerda da câmera, seguindo o robô."""
        x_robo = self.telemetria.get("x", 0.0) if self.telemetria else 0.0
        cam = x_robo - METROS_VISIVEIS / 2.0
        return max(0.0, cam)

    def _tela_x(self, x_m, cam):
        return int(MARGEM + (x_m - cam) * ESCALA_X)

    def _tela_y_teto(self, y_cm):
        # y maior (buraco) -> teto mais ALTO na tela; y menor (saliência) -> mais baixo.
        return int(TETO_TELA_BASE - (y_cm - TETO_BASE) * ESCALA_Y)

    def _desloc_y(self, x_m):
        """Deslocamento vertical (px) do túnel em x, relativo ao piso sob o robô.
        Mantém o robô numa altura fixa (CHAO_TELA) e faz o túnel inclinar ao redor:
        trechos mais altos que o robô sobem na tela (y menor), mais baixos descem."""
        x_robo = self.telemetria.get("x", 0.0) if self.telemetria else 0.0
        return -(altura_piso(x_m) - altura_piso(x_robo)) * ESCALA_PISO

    # ------------------------------------------------------------------ #
    #  Desenho
    # ------------------------------------------------------------------ #
    def _desenhar(self):
        self.tela.fill(COR_FUNDO)
        cam = self._camera_x()

        # --- Teto real ao fundo (ground truth), deslocado pela inclinação do piso ---
        if TEM_GROUND_TRUTH:
            pts_real = []
            x = cam
            while x <= cam + METROS_VISIVEIS:
                y = self._tela_y_teto(perfil_teto(x)) + self._desloc_y(x)
                pts_real.append((self._tela_x(x, cam), int(y)))
                x += 0.25
            if len(pts_real) > 1:
                pygame.draw.lines(self.tela, COR_REAL, False, pts_real, 2)

        # --- Massa de rocha do teto (polígono do topo da tela até a linha do teto) ---
        # O teto acompanha a inclinação do piso (o túnel inteiro tomba na rampa).
        pts_teto = [(MARGEM, 0), (LARGURA - MARGEM, 0)]
        x = cam + METROS_VISIVEIS
        while x >= cam:
            yt = perfil_teto(x) if TEM_GROUND_TRUTH else TETO_BASE
            y = self._tela_y_teto(yt) + self._desloc_y(x)
            pts_teto.append((self._tela_x(x, cam), int(y)))
            x -= 0.25
        pygame.draw.polygon(self.tela, COR_ROCHA_ESC, pts_teto)

        # --- Chão seguindo o perfil de inclinação (a rampa visual) ---
        # Linha de superfície do piso e massa de rocha abaixo dela.
        pts_sup = []
        x = cam
        while x <= cam + METROS_VISIVEIS:
            y = CHAO_TELA + self._desloc_y(x)
            pts_sup.append((self._tela_x(x, cam), int(y)))
            x += 0.25
        pts_chao = list(pts_sup)
        pts_chao.append((self._tela_x(cam + METROS_VISIVEIS, cam), ALTURA))
        pts_chao.append((self._tela_x(cam, cam), ALTURA))
        pygame.draw.polygon(self.tela, COR_CHAO, pts_chao)
        if len(pts_sup) > 1:
            pygame.draw.lines(self.tela, COR_CHAO_LINHA, False, pts_sup, 2)

        # --- Perfil MEDIDO pelo robô (a reconstrução em tempo real) ---
        metros = sorted(m for m in self.perfil_medido
                        if cam - 1 <= m <= cam + METROS_VISIVEIS + 1)
        pts_med = [(self._tela_x(m, cam),
                    int(self._tela_y_teto(self.perfil_medido[m]) + self._desloc_y(m)))
                   for m in metros]
        if len(pts_med) > 1:
            pygame.draw.lines(self.tela, COR_PERFIL, False, pts_med, 3)
        for p in pts_med:
            pygame.draw.circle(self.tela, COR_PERFIL, p, 3)

        # --- Robô (fica em CHAO_TELA, inclinado pelo pitch; o túnel tomba ao redor) ---
        self._desenhar_robo(cam)

        # --- Painel de texto (HUD) ---
        self._desenhar_hud()

        pygame.display.flip()

    def _desenhar_robo(self, cam):
        t = self.telemetria
        x_robo = t.get("x", 0.0) if t else 0.0
        inspecao = t.get("inspecao", False) if t else False
        camera = t.get("liga_camera", False) if t else False
        pitch = t.get("inclinacao", 0.0) if t else 0.0   # graus (IMU)

        rx = self._tela_x(x_robo, cam)
        ry = CHAO_TELA

        cor = COR_ROBO_INSP if (inspecao or camera) else COR_ROBO

        # Desenha o robô em uma superfície própria e a rotaciona pelo pitch (declive),
        # de modo que o carrinho aparece inclinado na subida/descida.
        larg, alt = 64, 52
        surf = pygame.Surface((larg, alt), pygame.SRCALPHA)
        cx, cy = larg // 2, alt - 14   # referência: eixo das rodas
        pygame.draw.rect(surf, cor, (cx - 22, cy - 24, 44, 22), border_radius=4)  # corpo
        pygame.draw.circle(surf, (30, 30, 30), (cx - 12, cy), 6)  # roda traseira
        pygame.draw.circle(surf, (30, 30, 30), (cx + 12, cy), 6)  # roda dianteira
        pygame.draw.rect(surf, (40, 40, 40), (cx - 5, cy - 32, 10, 8))  # câmera no topo

        # pygame rotaciona no sentido anti-horário: pitch positivo (subida) ergue a
        # frente (lado direito) do robô, como esperado numa rampa de subida.
        rot = pygame.transform.rotate(surf, pitch)
        rect = rot.get_rect(center=(rx, ry - 14))
        self.tela.blit(rot, rect.topleft)

        # Feixe da câmera quando inspecionando
        if inspecao or camera:
            beam = [(rx, ry - 30), (rx - 14, self._tela_y_teto(TETO_BASE) + 20),
                    (rx + 14, self._tela_y_teto(TETO_BASE) + 20)]
            superficie = pygame.Surface((LARGURA, ALTURA), pygame.SRCALPHA)
            pygame.draw.polygon(superficie, (235, 200, 80, 60), beam)
            self.tela.blit(superficie, (0, 0))
            txt = self.fonte_pq.render("INSPECIONANDO", True, COR_DESTAQUE)
            self.tela.blit(txt, (rx - 45, ry - 60))

    def _desenhar_hud(self):
        t = self.telemetria
        # Faixa de status no topo
        pygame.draw.rect(self.tela, (28, 28, 36), (0, 0, LARGURA, 34))
        if t is None:
            txt = "Aguardando telemetria..." if self.conectado else "Conectando ao broker..."
            self.tela.blit(self.fonte.render(txt, True, COR_TEXTO), (12, 7))
            return

        modo = t.get("modo", "—").upper()
        inc = t.get("inclinacao", 0.0)
        seta = "/" if inc > 0.5 else ("\\" if inc < -0.5 else "-")
        info = (f"Posição: {t.get('x', 0):.1f} m    "
                f"Teto: {t.get('y', 0)} cm    "
                f"Veloc.: {t.get('velocidade', 0):.1f} m/s    "
                f"Inclin.: {inc:+.1f} {seta}    "
                f"Confiança: {t.get('confianca', 0)}%    "
                f"Modo: {modo}")
        self.tela.blit(self.fonte.render(info, True, COR_TEXTO), (12, 7))

        # Legenda no rodapé
        leg_y = ALTURA - 26
        pygame.draw.line(self.tela, COR_PERFIL, (12, leg_y), (40, leg_y), 3)
        self.tela.blit(self.fonte_pq.render("perfil medido (lidar)", True, COR_TEXTO), (46, leg_y - 8))
        if TEM_GROUND_TRUTH:
            pygame.draw.line(self.tela, COR_REAL, (230, leg_y), (258, leg_y), 2)
            self.tela.blit(self.fonte_pq.render("teto real (simulador)", True, COR_TEXTO), (264, leg_y - 8))

    # ------------------------------------------------------------------ #
    #  Loop principal
    # ------------------------------------------------------------------ #
    def executar(self):
        rodando = True
        while rodando:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    rodando = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    rodando = False

            self._desenhar()
            self.relogio.tick(FPS)

        self.client.loop_stop()
        self.client.disconnect()
        pygame.quit()
        sys.exit()


def main():
    Visualizacao().executar()


if __name__ == "__main__":
    main()