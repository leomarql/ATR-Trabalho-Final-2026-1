# Contrato da API MQTT — Etapa 2

Sistema de inspeção autônoma de túneis — Automação em Tempo Real (2026/1).

Este documento é a **referência única** dos tópicos MQTT e dos formatos de
mensagem trocados entre os processos do sistema. Nenhum dos lados deve mudar
tópico, nome de campo ou tipo sem atualizar este arquivo primeiro.

## Visão geral

O sistema fecha um laço entre processos que conversam pelo broker. Cada um tem
um papel bem definido:

- **Robô embarcado (C++)** — dono dos **atuadores** e da **telemetria**.
- **Simulador físico (Python)** — dono dos **sensores** (substitui o `mock.cpp`).
- **Operação Remota (Python)** — dona dos **comandos** do operador.
- **Simulação Visual (Python)** — assina a **telemetria** e a **inspeção visual**
  para desenhar o túnel e o robô.
- **Inspeção Visual / YOLO (Python, EXTRA)** — assina a **telemetria** (gatilho da
  câmera) e publica o resultado da detecção em `robo/inspecao_visual`.

Fluxo do laço fechado:

```
Operação Remota --robo/comandos-->  C++  --robo/atuadores-->  Simulador
                                     ^                              |
                                     |                              |
        robo/telemetria  <-----------+        robo/sensores <-------+
        (Operação Remota + GUI sim)             (C++ lê sensores)
```

## Broker

- **Implementação:** Eclipse Mosquitto
- **Endereço:** `localhost`
- **Porta:** `1883`
- **Autenticação:** nenhuma (rede local, ambiente de desenvolvimento)
- **QoS padrão:** 0 (ajustar para 1 nos comandos se houver perda perceptível)

Todos os payloads são **JSON em UTF-8**. Nada de strings soltas do tipo
`"VELOCIDADE=2"`.

## Convenção de unidades (padrão do projeto)

Padrão fixo, consistente com o código embarcado e com o `log_inspecao.csv`:

| Grandeza            | Unidade | Observação                                  |
|---------------------|---------|---------------------------------------------|
| Posição X           | metros  | eixo longitudinal do túnel (`Posicao_X_m`)  |
| Posição Y / lidar   | cm      | distância vertical ao teto (`Posicao_Y_cm`) |
| Velocidade          | m/s     | setpoint e velocidade medida                |
| Aceleração/atuação  | %       | -100 a 100 (sinal PWM)                       |

Convenção de sentido no eixo X: **direita = sentido positivo (avanço, +X)**,
**esquerda = sentido negativo (recuo, -X)**.

## Tópicos

### 1. `robo/atuadores`

Publicado pelo **C++**, assinado pelo **Simulador**.
É o que fecha o laço: o simulador precisa da aceleração para aplicar Newton.

```json
{ "aceleracao": 0 }
```

| Campo        | Tipo | Faixa      | Origem (variável C++) |
|--------------|------|------------|-----------------------|
| `aceleracao` | int  | -100 a 100 | `o_aceleracao`        |

### 2. `robo/sensores`

Publicado pelo **Simulador**, assinado pelo **C++**.
Substitui o `mock.cpp`: alimenta os sensores do robô com os valores gerados
pela física + ruído de medição.

```json
{ "encoder": false, "sentido": 1, "lidar": 200, "imu_pitch": 0.0 }
```

| Campo       | Tipo   | Faixa             | Destino (variável C++) |
|-------------|--------|-------------------|------------------------|
| `encoder`   | bool   | true/false        | `i_encoder`            |
| `sentido`   | int    | -1, 0, +1         | `i_sentido`            |
| `lidar`     | int    | cm (ex. 200)      | `i_lidar`              |
| `imu_pitch` | double | graus (ex. ±6)    | `i_imu_pitch`          |

O campo `sentido` indica a direção do movimento (+1 avanço, -1 recuo, 0 parado),
análogo ao sinal que um encoder em quadratura forneceria. O encoder permanece
**binário** (1 troca de estado por metro, conforme o enunciado); o `sentido` é a
informação adicional que permite à odometria contar a distância com o sinal
correto (avanço soma, recuo subtrai), viabilizando o comando `c_esquerda`.

O campo `imu_pitch` é a inclinação (pitch) do piso medida por uma **IMU simulada**
(funcionalidade EXTRA: túnel com declive). É independente do `lidar` (que mede o
teto): o IMU mede a rampa do piso. O controle usa essa leitura para compensar a
gravidade da rampa via feed-forward.

### 3. `robo/telemetria`

Publicado pelo **C++**, assinado pela **Operação Remota** e pela **GUI da
simulação**. É o "estado do robô" para exibição na tela.

```json
{
  "x": 0.0,
  "y": 200,
  "confianca": 100,
  "velocidade": 0.0,
  "modo": "auto",
  "inspecao": false,
  "liga_camera": false,
  "inclinacao": 0.0
}
```

| Campo         | Tipo   | Faixa/valores      | Origem (variável C++)        |
|---------------|--------|--------------------|------------------------------|
| `x`           | double | metros             | posição da odometria         |
| `y`           | int    | cm                 | leitura do teto (`i_lidar`)  |
| `confianca`   | int    | 0 a 100            | calculada no coletor         |
| `velocidade`  | double | m/s                | `velocidade_atual`           |
| `modo`        | string | `"auto"`/`"manual"`| `e_automatico`               |
| `inspecao`    | bool   | true/false         | `e_inspecao`                 |
| `liga_camera` | bool   | true/false         | `o_liga_camera`              |
| `inclinacao`  | double | graus              | `i_imu_pitch` (IMU)          |

### 4. `robo/comandos`

Publicado pela **Operação Remota**, assinado pelo **C++**.
Formato `{comando, valor}` para acomodar comandos heterogêneos da Tabela 2.

```json
{ "comando": "set_modo", "valor": "auto" }
```

| Mensagem                                          | Efeito no C++                         | Comando da Tabela 2          |
|---------------------------------------------------|---------------------------------------|------------------------------|
| `{"comando":"set_modo","valor":"auto"}`           | `e_automatico = true`                 | `c_automatico`               |
| `{"comando":"set_modo","valor":"manual"}`         | `e_automatico = false`                | `c_man`                      |
| `{"comando":"set_velocidade","valor":5.0}`        | alimenta `j_sp_velocidade`            | `j_sp_velocidade`            |
| `{"comando":"direcao","valor":"direita"}`         | acelera no sentido +X                 | `c_direita`                  |
| `{"comando":"direcao","valor":"esquerda"}`        | acelera no sentido -X                 | `c_esquerda`                 |
| `{"comando":"direcao","valor":"parar"}`           | para o robô (setpoint 0)              | `c_para`                     |
| `{"comando":"set_limiar","valor":15}`             | escreve em `limiar_anomalia`          | limiar configurável (item 4) |

#### Semântica dos comandos de direção

Os comandos de direção operam sobre o **modo manual** e definem o setpoint de
velocidade conforme o sentido:

- `direcao: "direita"` → setpoint de velocidade **positivo** (avanço, +X).
- `direcao: "esquerda"` → setpoint de velocidade **negativo** (recuo, -X).
- `direcao: "parar"` → setpoint **0**.

O módulo do setpoint usado por `direita`/`esquerda` segue o último
`set_velocidade` recebido (padrão inicial: 5.0 m/s, caso nenhum tenha sido
enviado). No modo automático, esses comandos são ignorados — quem manda é a
lógica autônoma.

> **Decisão de projeto (recuo / sentido):** o recuo (-X) exige que a odometria
> saiba a direção do movimento. Como um encoder binário simples não carrega
> direção (isso exigiria um encoder em quadratura), o **simulador publica o campo
> `sentido`** em `robo/sensores` (+1/-1/0), e a odometria soma ou subtrai 1 metro
> por troca de estado conforme esse sinal (`distancia_total += i_sentido`). Assim
> o encoder permanece binário (1 troca/metro) e a distância continua sendo
> calculada a partir dele — em conformidade com o enunciado — enquanto o comando
> `c_esquerda` passa a mover o robô para trás de fato. A velocidade estimada por
> janela de tempo fica naturalmente negativa no recuo, sem tratamento adicional.

### 5. `robo/inspecao_visual` (EXTRA: YOLO)

Publicado pelo **serviço de inspeção visual** (`inspecao_visual.py`), assinado pela
**Simulação Visual**. Resultado da detecção de objetos por YOLO quando a câmera é
acionada em uma anomalia (gatilho: borda de subida de `liga_camera` na telemetria).

```json
{
  "x": 9.0,
  "n": 2,
  "objetos": [
    { "classe": "backpack", "conf": 0.71 },
    { "classe": "bottle",   "conf": 0.55 }
  ],
  "imagem": "insp_x0009.0.jpg"
}
```

| Campo      | Tipo   | Descrição                                            |
|------------|--------|------------------------------------------------------|
| `x`        | double | posição (m) em que a câmera foi acionada             |
| `n`        | int    | número de objetos detectados                         |
| `objetos`  | lista  | cada item: `{classe (string), conf (0..1)}`          |
| `imagem`   | string | nome do arquivo anotado salvo em `inspecao_resultados/` |

## Frequência de publicação

- **`robo/telemetria`** — publicada a cada novo registro do coletor (ou seja, a
  cada leitura do lidar consumida, ~100 ms).
- **`robo/sensores`** — publicada na cadência do passo de integração da física
  do simulador.
- **`robo/atuadores`** — publicada quando `o_aceleracao` muda (ou em cadência
  fixa curta, p.ex. junto ao ciclo de controle de 80 ms).
- **`robo/comandos`** — publicada por evento (clique do operador na GUI).

## Cobertura das Tabelas 1 e 2 do enunciado

Checagem de que toda variável de E/S tem um caminho na rede:

**Tabela 1 (sensores e atuadores)**

| Variável        | Tópico             | Sentido        |
|-----------------|--------------------|----------------|
| `i_encoder`     | `robo/sensores`    | Simulador → C++|
| `i_sentido`     | `robo/sensores`    | Simulador → C++|
| `i_imu_pitch`   | `robo/sensores`    | Simulador → C++|
| `i_lidar`       | `robo/sensores`    | Simulador → C++|
| `o_liga_camera` | `robo/telemetria`  | C++ → GUIs     |
| `o_aceleracao`  | `robo/atuadores`   | C++ → Simulador|

**Tabela 2 (estados e comandos)**

| Variável         | Tópico            | Sentido            |
|------------------|-------------------|--------------------|
| `e_inspecao`     | `robo/telemetria` | C++ → GUIs         |
| `e_automatico`   | `robo/telemetria` | C++ → GUIs         |
| `c_automatico`   | `robo/comandos`   | Op. Remota → C++   |
| `c_man`          | `robo/comandos`   | Op. Remota → C++   |
| `j_sp_velocidade`| `robo/comandos`   | Op. Remota → C++   |
| `c_direita`      | `robo/comandos`   | Op. Remota → C++   |
| `c_esquerda`     | `robo/comandos`   | Op. Remota → C++   |
| `c_para`         | `robo/comandos`   | Op. Remota → C++   |

## Como testar cada ponta isoladamente

Antes de ligar tudo, cada lado valida sozinho via terminal:

```bash
# Assinar um tópico e ver o que chega
mosquitto_sub -t "robo/telemetria" -v

# Publicar um comando manualmente
mosquitto_pub -t "robo/comandos" -m '{"comando":"set_modo","valor":"manual"}'
```