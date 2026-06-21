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
import random
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
TOPICO_INSPECAO = "robo/inspecao_visual"   # resultado do YOLO (EXTRA)
TOPICO_COMANDOS = "robo/comandos"          # saída: comandos do operador (teclado)

# --- Configuração da janela ---
# A janela tem duas regiões empilhadas: em cima a CENA do túnel (robô andando) e
# embaixo uma faixa com o GRÁFICO do perfil do teto (o "gráfico de dados LIDAR
# processados" da Figura 2 do enunciado), com eixos retos (sem a distorção da rampa).
LARGURA = 960
CENA_ALTURA = 540               # altura da cena do túnel (parte de cima)
GRAFICO_ALTURA = 170            # altura da faixa do gráfico (parte de baixo)
ALTURA = CENA_ALTURA + GRAFICO_ALTURA
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
COR_FUNDO_TOPO = (10, 10, 16)       # gradiente de fundo (topo)
COR_FUNDO_BASE = (30, 27, 36)       # gradiente de fundo (base)
COR_ROCHA = (82, 71, 60)            # realce claro da rocha
COR_ROCHA_ESC = (54, 47, 40)        # massa de rocha do teto
COR_ROCHA_MAIS_ESC = (38, 33, 28)   # pontos escuros (textura)
COR_ROCHA_RIM = (104, 90, 76)       # linha de superfície do teto (realce)
COR_CHAO = (64, 56, 48)             # massa de rocha do chão
COR_CHAO_LINHA = (112, 99, 84)      # linha de superfície do piso (rampa)
COR_PERFIL = (95, 212, 135)         # perfil medido (verde)
COR_REAL = (96, 96, 122)            # teto real ao fundo (cinza-azulado)
COR_ROBO = (72, 152, 212)           # corpo do robô (azul-petróleo)
COR_ROBO_CLARO = (124, 198, 240)    # painel superior
COR_ROBO_ESC = (38, 92, 138)        # contorno/cabeça
COR_LENTE = (150, 140, 92)          # lente da câmera (apagada)
COR_LENTE_INSP = (255, 238, 150)    # lente da câmera (brilhando na inspeção)
COR_ROBO_INSP = (235, 90, 80)       # (mantido p/ compatibilidade)
COR_TEXTO = (226, 226, 230)
COR_DESTAQUE = (240, 206, 92)
# Painel do gráfico de perfil (Figura 2)
COR_GRAF_FUNDO = (16, 20, 28)       # fundo da faixa do gráfico
COR_GRAF_EIXO = (90, 95, 110)       # eixos e moldura
COR_GRAF_GRADE = (40, 44, 54)       # linhas de grade
COR_GRAF_BASE = (70, 76, 92)        # linha do teto nominal (referência)
COR_GRAF_ROBO = (240, 206, 92)      # marcador da posição atual do robô

# --- Textura da rocha: pontos e fissuras em coordenadas do MUNDO ---
# Gerados uma única vez (RNG com semente fixa -> determinístico, não cintila).
# Cada ponto rola e tomba junto com o túnel ao ser projetado na tela.
_rng = random.Random(42)
_DETALHES_ROCHA = []   # (x_mundo, profundidade_px, raio, cor, lado)
for _ in range(320):
    _x = _rng.uniform(0, 140)
    _lado = "teto" if _rng.random() < 0.5 else "chao"
    _prof = _rng.uniform(10, 130)     # distância (px) para dentro da rocha
    _raio = _rng.randint(1, 3)
    _cor = _rng.choice([COR_ROCHA_MAIS_ESC, COR_ROCHA_MAIS_ESC, COR_ROCHA])
    _DETALHES_ROCHA.append((_x, _prof, _raio, _cor, _lado))


class Visualizacao:
    def __init__(self):
        self.telemetria = None       # última telemetria recebida (dict)
        self.perfil_medido = {}      # {metro(int): y_cm} reconstruído pelas medições
        self.conectado = False
        self.ultima_deteccao = None  # último resultado do YOLO (dict)
        self.t_deteccao = 0          # instante (ms) da última detecção, p/ esmaecer
        self.teclas_pressionadas = set()  # keycodes pressionados (teclado visual)

        # --- pygame ---
        pygame.init()
        self.tela = pygame.display.set_mode((LARGURA, ALTURA))
        pygame.display.set_caption("Simulação Visual — Robô de Inspeção de Túneis")
        self.fonte = pygame.font.SysFont("Arial", 18)
        self.fonte_pq = pygame.font.SysFont("Arial", 14)
        self.relogio = pygame.time.Clock()
        self.fundo = self._criar_fundo()   # gradiente pré-renderizado

        self._iniciar_mqtt()

    def _criar_fundo(self):
        """Pré-renderiza um gradiente vertical de fundo (feito uma vez)."""
        fundo = pygame.Surface((LARGURA, ALTURA))
        for y in range(ALTURA):
            t = y / ALTURA
            cor = tuple(int(COR_FUNDO_TOPO[i] + (COR_FUNDO_BASE[i] - COR_FUNDO_TOPO[i]) * t)
                        for i in range(3))
            pygame.draw.line(fundo, cor, (0, y), (LARGURA, y))
        return fundo

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
        client.subscribe(TOPICO_INSPECAO)
        print(f"[VIS] Conectado ao broker. Assinando {TOPICO_TELEMETRIA} e {TOPICO_INSPECAO}")

    def _enviar_comando(self, comando, valor):
        """Publica um comando no MESMO tópico/formato da Operação Remota.
        A visualização vira um segundo emissor de comandos (pelo teclado), sem
        alterar o contrato: o C++ trata a mensagem igual à da Operação Remota."""
        try:
            self.client.publish(TOPICO_COMANDOS,
                                json.dumps({"comando": comando, "valor": valor}))
        except Exception as e:
            print(f"[VIS] Falha ao publicar comando: {e}")

    # ------------------------------------------------------------------ #
    #  Teclado (controle manual + feedback visual)
    # ------------------------------------------------------------------ #
    def _tecla_baixo(self, key):
        """Tecla pressionada: registra para o teclado visual e publica o comando.
        Modelo 'segurar para andar': a seta inicia o movimento; ao soltar, para."""
        self.teclas_pressionadas.add(key)
        if key == pygame.K_LEFT:
            self._enviar_comando("direcao", "esquerda")
        elif key == pygame.K_RIGHT:
            self._enviar_comando("direcao", "direita")
        elif key in (pygame.K_SPACE, pygame.K_DOWN):
            self._enviar_comando("direcao", "parar")
        elif key == pygame.K_a:
            self._enviar_comando("set_modo", "auto")
        elif key == pygame.K_m:
            self._enviar_comando("set_modo", "manual")

    def _tecla_cima(self, key):
        """Tecla solta: atualiza o teclado visual. Ao soltar uma seta, para o robô
        — a menos que a seta oposta ainda esteja pressionada (segue naquela direção)."""
        self.teclas_pressionadas.discard(key)
        if key == pygame.K_LEFT:
            if pygame.K_RIGHT in self.teclas_pressionadas:
                self._enviar_comando("direcao", "direita")
            else:
                self._enviar_comando("direcao", "parar")
        elif key == pygame.K_RIGHT:
            if pygame.K_LEFT in self.teclas_pressionadas:
                self._enviar_comando("direcao", "esquerda")
            else:
                self._enviar_comando("direcao", "parar")


    def _on_message(self, client, userdata, msg):
        try:
            dados = json.loads(msg.payload.decode())
        except Exception as e:
            print(f"[VIS] Mensagem ignorada: {e}")
            return

        if msg.topic == TOPICO_INSPECAO:
            # Resultado do YOLO (EXTRA): guarda para exibir no painel.
            self.ultima_deteccao = dados
            self.t_deteccao = pygame.time.get_ticks()
            return

        # Telemetria: reconstrói o perfil do teto (leitura y por posição x).
        # Chaveia a 0,5 m (resolução do encoder) para preservar os pontos finos.
        self.telemetria = dados
        x = dados.get("x", 0.0)
        y = dados.get("y", TETO_BASE)
        self.perfil_medido[round(x * 2) / 2.0] = y

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
        self.tela.blit(self.fundo, (0, 0))   # fundo em gradiente (pré-renderizado)
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
        pts_sup_teto = []   # linha de superfície do teto (para o realce)
        x = cam
        while x <= cam + METROS_VISIVEIS:
            yt = perfil_teto(x) if TEM_GROUND_TRUTH else TETO_BASE
            y = self._tela_y_teto(yt) + self._desloc_y(x)
            pts_sup_teto.append((self._tela_x(x, cam), int(y)))
            x += 0.25
        pts_teto = [(MARGEM, 0), (LARGURA - MARGEM, 0)] + list(reversed(pts_sup_teto))
        pygame.draw.polygon(self.tela, COR_ROCHA_ESC, pts_teto)
        if len(pts_sup_teto) > 1:   # realce na superfície do teto
            pygame.draw.lines(self.tela, COR_ROCHA_RIM, False, pts_sup_teto, 2)

        # --- Chão seguindo o perfil de inclinação (a rampa visual) ---
        pts_sup = []
        x = cam
        while x <= cam + METROS_VISIVEIS:
            y = CHAO_TELA + self._desloc_y(x)
            pts_sup.append((self._tela_x(x, cam), int(y)))
            x += 0.25
        pts_chao = list(pts_sup)
        pts_chao.append((self._tela_x(cam + METROS_VISIVEIS, cam), CENA_ALTURA))
        pts_chao.append((self._tela_x(cam, cam), CENA_ALTURA))
        pygame.draw.polygon(self.tela, COR_CHAO, pts_chao)
        if len(pts_sup) > 1:
            pygame.draw.lines(self.tela, COR_CHAO_LINHA, False, pts_sup, 2)

        # --- Textura da rocha (pontos do mundo, projetados; rolam e tombam junto) ---
        self._desenhar_textura_rocha(cam)

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

        # --- Painel de detecção do YOLO (EXTRA: inspeção visual) ---
        self._desenhar_painel_yolo()

        # --- Painel de texto (HUD) ---
        self._desenhar_hud()

        # --- Teclado visual (comandos na tela + feedback de tecla pressionada) ---
        self._desenhar_teclado()

        # --- Faixa inferior: gráfico do perfil do teto (Figura 2 do enunciado) ---
        self._desenhar_grafico(cam)

        pygame.display.flip()

    def _desenhar_textura_rocha(self, cam):
        """Desenha os pontos de textura da rocha que estão visíveis, projetados
        no teto e no chão com o mesmo deslocamento de inclinação do túnel."""
        for x, prof, raio, cor, lado in _DETALHES_ROCHA:
            if not (cam - 1 <= x <= cam + METROS_VISIVEIS + 1):
                continue
            sx = self._tela_x(x, cam)
            desloc = self._desloc_y(x)
            if lado == "teto":
                base_y = self._tela_y_teto(perfil_teto(x)) + desloc
                sy = base_y - prof          # para dentro da rocha (acima da superfície)
                if sy < 2:
                    continue
            else:
                base_y = CHAO_TELA + desloc
                sy = base_y + prof          # para dentro do chão (abaixo da superfície)
                if sy > CENA_ALTURA - 2:
                    continue
            pygame.draw.circle(self.tela, cor, (sx, int(sy)), raio)

    def _desenhar_robo(self, cam):
        t = self.telemetria
        x_robo = t.get("x", 0.0) if t else 0.0
        inspecao = t.get("inspecao", False) if t else False
        camera = t.get("liga_camera", False) if t else False
        pitch = t.get("inclinacao", 0.0) if t else 0.0   # graus (IMU)
        insp = inspecao or camera

        rx = self._tela_x(x_robo, cam)
        ry = CHAO_TELA

        # --- Sombra elíptica no chão (semi-transparente; não rotaciona) ---
        sombra = pygame.Surface((78, 18), pygame.SRCALPHA)
        pygame.draw.ellipse(sombra, (0, 0, 0, 90), sombra.get_rect())
        self.tela.blit(sombra, (rx - 39, ry - 6))

        # --- Sprite do robô (desenhado em superfície própria e rotacionado) ---
        W, H = 96, 84
        base = H - 20                 # linha de contato das esteiras (toca o chão)
        surf = pygame.Surface((W, H), pygame.SRCALPHA)
        cx = W // 2

        # Esteira tipo rover (retângulo arredondado escuro + rodinhas internas)
        pygame.draw.rect(surf, (34, 34, 40), (cx - 30, base - 4, 60, 16), border_radius=8)
        for wx in (-19, -6, 7, 20):
            pygame.draw.circle(surf, (88, 88, 96), (cx + wx, base + 4), 4)
            pygame.draw.circle(surf, (18, 18, 22), (cx + wx, base + 4), 4, 1)

        # Corpo arredondado (dois tons) + painel superior claro
        corpo = pygame.Rect(cx - 27, base - 32, 54, 28)
        pygame.draw.rect(surf, COR_ROBO, corpo, border_radius=10)
        pygame.draw.rect(surf, COR_ROBO_ESC, corpo, 2, border_radius=10)
        pygame.draw.rect(surf, COR_ROBO_CLARO, (cx - 21, base - 30, 42, 9), border_radius=5)

        # Antena com ponta luminosa (na traseira / lado esquerdo)
        pygame.draw.line(surf, (175, 175, 185), (cx - 18, base - 32), (cx - 18, base - 50), 2)
        pygame.draw.circle(surf, COR_DESTAQUE, (cx - 18, base - 52), 3)

        # Cabeça/sensor (na frente / lado direito) que abriga a lente
        pygame.draw.rect(surf, COR_ROBO_ESC, (cx + 4, base - 46, 22, 20), border_radius=6)

        # Lente da câmera — brilha durante a inspeção (com halo)
        lente = (cx + 17, base - 36)
        if insp:
            halo = pygame.Surface((44, 44), pygame.SRCALPHA)
            pygame.draw.circle(halo, (255, 232, 130, 95), (22, 22), 18)
            pygame.draw.circle(halo, (255, 244, 180, 70), (22, 22), 10)
            surf.blit(halo, (lente[0] - 22, lente[1] - 22))
        pygame.draw.circle(surf, COR_LENTE_INSP if insp else COR_LENTE, lente, 6)
        pygame.draw.circle(surf, (255, 255, 255), (lente[0] - 2, lente[1] - 2), 2)  # reflexo
        pygame.draw.circle(surf, (16, 16, 20), lente, 6, 1)

        # Rotaciona pelo pitch (anti-horário: subida ergue a frente) e ancora as
        # esteiras no chão (o ponto 'base' da superfície fica em ry = CHAO_TELA).
        rot = pygame.transform.rotate(surf, pitch)
        rect = rot.get_rect(center=(rx, ry + (H // 2 - base)))
        self.tela.blit(rot, rect.topleft)

        # --- Feixe da câmera quando inspecionando (a partir da lente, rumo ao teto) ---
        if insp:
            topo_y = self._tela_y_teto(TETO_BASE) + self._desloc_y(x_robo)
            beam = pygame.Surface((LARGURA, CENA_ALTURA), pygame.SRCALPHA)
            pygame.draw.polygon(beam, (255, 226, 120, 55),
                                [(rx + 6, ry - 44), (rx - 16, topo_y + 20),
                                 (rx + 30, topo_y + 20)])
            self.tela.blit(beam, (0, 0))
            txt = self.fonte_pq.render("INSPECIONANDO", True, COR_DESTAQUE)
            self.tela.blit(txt, (rx - 45, ry - 74))

    def _desenhar_painel_yolo(self):
        """Painel com o último resultado da inspeção visual por YOLO (EXTRA).
        Aparece no canto superior direito por alguns segundos após cada detecção."""
        d = self.ultima_deteccao
        if d is None:
            return
        # Esmaece o painel ~6s após a última detecção.
        idade = pygame.time.get_ticks() - self.t_deteccao
        if idade > 6000:
            return

        objetos = d.get("objetos", [])
        linhas = [f"YOLO @ x={d.get('x', 0)} m"]
        if objetos:
            for o in objetos[:6]:
                linhas.append(f"  {o['classe']}  {o['conf']:.2f}")
        else:
            linhas.append("  nenhum objeto detectado")

        larg_painel = 220
        alt_painel = 24 + 20 * len(linhas)
        px = LARGURA - larg_painel - 12
        py = 44

        painel = pygame.Surface((larg_painel, alt_painel), pygame.SRCALPHA)
        painel.fill((20, 24, 30, 210))
        pygame.draw.rect(painel, COR_DESTAQUE, painel.get_rect(), 2)
        self.tela.blit(painel, (px, py))

        # Título destacado, itens em branco.
        self.tela.blit(self.fonte_pq.render(linhas[0], True, COR_DESTAQUE), (px + 10, py + 8))
        for i, ln in enumerate(linhas[1:], start=1):
            self.tela.blit(self.fonte_pq.render(ln, True, COR_TEXTO), (px + 10, py + 8 + 20 * i))

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

        # Legenda no rodapé da cena (logo acima da faixa do gráfico)
        leg_y = CENA_ALTURA - 16
        pygame.draw.line(self.tela, COR_PERFIL, (12, leg_y), (40, leg_y), 3)
        self.tela.blit(self.fonte_pq.render("perfil medido (lidar)", True, COR_TEXTO), (46, leg_y - 8))
        if TEM_GROUND_TRUTH:
            pygame.draw.line(self.tela, COR_REAL, (230, leg_y), (258, leg_y), 2)
            self.tela.blit(self.fonte_pq.render("teto real (simulador)", True, COR_TEXTO), (264, leg_y - 8))

    def _desenhar_teclado(self):
        """Teclado visual (canto inferior direito da cena): mostra os comandos
        disponíveis (item 7 do enunciado) e DESTACA a tecla que está sendo
        pressionada no teclado real, dando feedback imediato ao operar o robô."""
        PW, PH = 264, 96
        PX, PY = LARGURA - PW - 14, CENA_ALTURA - PH - 20  # canto inferior direito

        # Sombra do painel (profundidade) + painel arredondado semitransparente
        sombra = pygame.Surface((PW + 8, PH + 8), pygame.SRCALPHA)
        pygame.draw.rect(sombra, (0, 0, 0, 90), sombra.get_rect(), border_radius=10)
        self.tela.blit(sombra, (PX - 2, PY))
        painel = pygame.Surface((PW, PH), pygame.SRCALPHA)
        pygame.draw.rect(painel, (22, 26, 33, 222), painel.get_rect(), border_radius=10)
        pygame.draw.rect(painel, (70, 76, 92), painel.get_rect(), 1, border_radius=10)
        self.tela.blit(painel, (PX, PY))

        # Título e rótulos de grupo
        self.tela.blit(self.fonte_pq.render("Controle por teclado", True, COR_TEXTO),
                       (PX + 12, PY + 7))
        self.tela.blit(self.fonte_pq.render("MODO", True, COR_GRAF_EIXO), (PX + 14, PY + 26))
        self.tela.blit(self.fonte_pq.render("DIREÇÃO", True, COR_GRAF_EIXO), (PX + 124, PY + 26))

        def tecla(x, y, w, h, rotulo, ativa, legenda):
            """Desenha uma tecla com leve efeito 3D; se 'ativa', acende (âmbar)."""
            r = pygame.Rect(PX + x, PY + y + (1 if ativa else 0), w, h)  # 'afunda' ao apertar
            if ativa:
                # halo de brilho atrás da tecla pressionada
                halo = pygame.Surface((w + 14, h + 14), pygame.SRCALPHA)
                pygame.draw.rect(halo, (240, 206, 92, 70), halo.get_rect(), border_radius=8)
                self.tela.blit(halo, (r.x - 7, r.y - 7))
                cor_fundo, cor_borda, cor_txt = (240, 206, 92), (255, 244, 180), (26, 26, 26)
            else:
                cor_fundo, cor_borda, cor_txt = (54, 59, 70), (98, 104, 120), (224, 224, 228)
            pygame.draw.rect(self.tela, cor_fundo, r, border_radius=6)
            # brilho superior (efeito de relevo) quando não pressionada
            if not ativa:
                pygame.draw.line(self.tela, (78, 84, 100), (r.x + 4, r.y + 2),
                                 (r.right - 4, r.y + 2), 1)
            pygame.draw.rect(self.tela, cor_borda, r, 2, border_radius=6)
            t = self.fonte.render(rotulo, True, cor_txt)
            self.tela.blit(t, t.get_rect(center=r.center))
            lg = self.fonte_pq.render(legenda, True, COR_TEXTO)
            self.tela.blit(lg, (PX + x + w // 2 - lg.get_width() // 2, PY + y + h + 2))

        P = self.teclas_pressionadas
        yk, h = 40, 28
        # Grupo MODO: A (auto), M (manual)
        tecla(10, yk, 32, h, "A", pygame.K_a in P, "auto")
        tecla(52, yk, 32, h, "M", pygame.K_m in P, "manual")
        # Grupo DIREÇÃO: <  espaço/parar  >
        tecla(120, yk, 30, h, "<", pygame.K_LEFT in P, "esquerda")
        tecla(156, yk, 58, h, "espaço",
              (pygame.K_SPACE in P or pygame.K_DOWN in P), "parar")
        tecla(220, yk, 30, h, ">", pygame.K_RIGHT in P, "direita")

    def _desenhar_grafico(self, cam):
        """Faixa inferior: gráfico do perfil do teto reconstruído (dados LIDAR
        processados), com eixos retos — X = posição (m), Y = altura do teto (cm).
        Equivale ao 'gráfico de dados LIDAR processados' da Figura 2 do enunciado.
        Diferente do perfil verde desenhado na cena (que segue a rampa e o túnel),
        aqui o perfil é plotado SEM distorção, como um gráfico técnico."""
        topo = CENA_ALTURA
        pygame.draw.rect(self.tela, COR_GRAF_FUNDO, (0, topo, LARGURA, GRAFICO_ALTURA))
        pygame.draw.line(self.tela, COR_GRAF_EIXO, (0, topo), (LARGURA, topo), 2)

        # Área de plotagem (margens internas para os rótulos dos eixos)
        pl_x0, pl_x1 = MARGEM, LARGURA - 16
        pl_y0, pl_y1 = topo + 26, ALTURA - 22
        larg_pl, alt_pl = pl_x1 - pl_x0, pl_y1 - pl_y0

        # Faixa de altura exibida (cm): cobre buracos (~285) e saliências (~140).
        Y_MIN, Y_MAX = 130, 290

        def y_para_tela(y_cm):
            y_cm = max(Y_MIN, min(Y_MAX, y_cm))
            frac = (y_cm - Y_MIN) / (Y_MAX - Y_MIN)
            return int(pl_y1 - frac * alt_pl)   # y maior (buraco) em cima

        def x_para_tela(x_m):
            return int(pl_x0 + (x_m - cam) / METROS_VISIVEIS * larg_pl)

        # Grade e rótulos do eixo Y (cm)
        for y_cm in (150, 200, 250):
            gy = y_para_tela(y_cm)
            pygame.draw.line(self.tela, COR_GRAF_GRADE, (pl_x0, gy), (pl_x1, gy), 1)
            self.tela.blit(self.fonte_pq.render(f"{y_cm}", True, COR_GRAF_EIXO), (6, gy - 8))
        # Linha do teto nominal (200 cm) em destaque
        gy0 = y_para_tela(TETO_BASE)
        pygame.draw.line(self.tela, COR_GRAF_BASE, (pl_x0, gy0), (pl_x1, gy0), 1)

        # Grade e rótulos do eixo X (m), a cada 5 m
        m = int(math.ceil(cam / 5.0) * 5)
        while m <= cam + METROS_VISIVEIS:
            gx = x_para_tela(m)
            pygame.draw.line(self.tela, COR_GRAF_GRADE, (gx, pl_y0), (gx, pl_y1), 1)
            self.tela.blit(self.fonte_pq.render(f"{m}", True, COR_GRAF_EIXO), (gx - 6, pl_y1 + 4))
            m += 5

        # Moldura dos eixos
        pygame.draw.line(self.tela, COR_GRAF_EIXO, (pl_x0, pl_y0), (pl_x0, pl_y1), 1)
        pygame.draw.line(self.tela, COR_GRAF_EIXO, (pl_x0, pl_y1), (pl_x1, pl_y1), 1)

        # Perfil medido (verde), plotado reto (X = posição, Y = altura)
        metros = sorted(mm for mm in self.perfil_medido
                        if cam <= mm <= cam + METROS_VISIVEIS)
        pts = [(x_para_tela(mm), y_para_tela(self.perfil_medido[mm])) for mm in metros]
        if len(pts) > 1:
            pygame.draw.lines(self.tela, COR_PERFIL, False, pts, 2)
        for p in pts:
            pygame.draw.circle(self.tela, COR_PERFIL, p, 2)

        # Marcador vertical da posição atual do robô
        x_robo = self.telemetria.get("x", 0.0) if self.telemetria else 0.0
        if cam <= x_robo <= cam + METROS_VISIVEIS:
            rx = x_para_tela(x_robo)
            pygame.draw.line(self.tela, COR_GRAF_ROBO, (rx, pl_y0), (rx, pl_y1), 1)

        # Título do painel
        titulo = self.fonte_pq.render(
            "Gráfico de dados LIDAR processados — perfil do teto (Y, cm) × posição (X, m)",
            True, COR_TEXTO)
        self.tela.blit(titulo, (pl_x0, topo + 7))

    # ------------------------------------------------------------------ #
    #  Loop principal
    # ------------------------------------------------------------------ #
    def executar(self):
        rodando = True
        while rodando:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    rodando = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        rodando = False
                    else:
                        self._tecla_baixo(ev.key)
                elif ev.type == pygame.KEYUP:
                    self._tecla_cima(ev.key)

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