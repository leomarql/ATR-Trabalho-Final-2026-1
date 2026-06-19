# Contrato da API MQTT — Etapa 2

Sistema de inspeção autônoma de túneis — Automação em Tempo Real (2026/1).

Este documento é a **referência única** dos tópicos MQTT e dos formatos de
mensagem trocados entre os três processos do sistema. Nenhum dos lados deve
mudar tópico, nome de campo ou tipo sem atualizar este arquivo primeiro.

## Visão geral

O sistema fecha um laço entre três processos que conversam pelo broker:

- **Robô embarcado (C++)** — dono dos **atuadores** e da **telemetria**.
- **Simulador físico (Python)** — dono dos **sensores** (substitui o `mock.cpp`).
- **Operação Remota (Python)** — dona dos **comandos** do operador.

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
{ "encoder": false, "lidar": 200 }
```

| Campo     | Tipo | Faixa        | Destino (variável C++) |
|-----------|------|--------------|------------------------|
| `encoder` | bool | true/false   | `i_encoder`            |
| `lidar`   | int  | cm (ex. 200) | `i_lidar`              |

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
  "liga_camera": false
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

> **Pendência de implementação (Fase 2/odometria):** o recuo (-X) exige suporte
> a sentido na física e na odometria. Hoje a odometria só incrementa distância
> (`distancia_total += 1.0`), sem noção de direção. Para `esquerda` funcionar de
> verdade, o simulador precisa informar o sentido do movimento e a odometria
> precisa somar **ou** subtrair conforme esse sentido. Registrar isso no
> relatório como decisão de projeto.

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