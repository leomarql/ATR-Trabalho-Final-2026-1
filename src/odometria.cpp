/* 
Odometria.cpp - Cálculo de Distância e Velocidade a partir do Encoder
O que faz: Roda assincronamente a cada 20ms. 
Lê o sensor do encoder e conta PASSO_ENCODER metros percorridos a CADA troca de
estado do encoder (0->1 ou 1->0). O enunciado sugere 1 m por troca, mas o professor
autorizou frequência maior; usamos 0,5 m para dobrar a resolução do mapeamento. O
SENTIDO do movimento (i_sentido, fornecido pelo simulador) define se a distância é
somada (avanço) ou subtraída (recuo), permitindo o comando "esquerda" (Tabela 2)
mover o robô para trás de fato.
A cada troca, atualiza a distância, estima a velocidade, abastece o controlador PID
e o Coletor, e expõe a posição X (posicao_x) para a telemetria MQTT.

Estimativa de velocidade por JANELA DE TEMPO: como o encoder dá apenas 1 pulso por
metro, derivar a distância a cada ciclo de 20ms produz um sinal "0 ou 50" (muito
ruidoso). Em vez disso, mede-se quantos metros foram percorridos nos últimos ~0,5s
e divide-se pelo tempo real decorrido — gerando uma velocidade estável e correta.
Como a distância agora pode diminuir (recuo), a velocidade fica naturalmente
negativa ao andar para trás, sem nenhum tratamento adicional. Isso NÃO altera a
semântica do encoder (binário, 1 troca/metro) nem o cálculo da distância a partir
dele, exigidos pelo enunciado.

Mede o próprio tempo de execução (WCET) para a análise de escalonabilidade.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <deque>
#include <utility>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"
#include "Profiler.hpp"

// Variáveis Globais (Vêm do main.cpp)
extern std::atomic<bool> i_encoder;
extern std::atomic<int> i_sentido;      // +1 avanço, -1 recuo, 0 parado
extern std::atomic<double> velocidade_atual;
extern std::atomic<double> posicao_x;   // exposta para a telemetria
extern BufferCompartilhado<double> buffer_distancia_coletor;
extern MedidorWCET wcet_odometria;      // medidor de tempo de execução

void callback_odometria(boost::asio::steady_timer& timer) {
    auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET (e timestamp do ciclo)

    // Variáveis estáticas mantêm o valor entre os ciclos
    static bool estado_anterior_encoder = false;
    static double distancia_total = 0.0;

    // Janela deslizante (tempo, distância) para a estimativa de velocidade
    static std::deque<std::pair<std::chrono::steady_clock::time_point, double>> janela;
    const auto JANELA_DUR = std::chrono::milliseconds(500);  // ~0,5s

    // Distância (m) por troca de estado do encoder. Deve casar com o PASSO_ENCODER
    // do simulador (python/simulador.py). O enunciado sugere 1 m, mas o professor
    // autorizou frequência maior; usamos 0,5 m para dobrar a resolução de mapeamento.
    const double PASSO_ENCODER = 0.5;

    bool estado_atual_encoder = i_encoder.load();

    // Conta PASSO_ENCODER metros a cada TROCA DE ESTADO (subida ou descida), conforme
    // o enunciado. O sentido define o sinal: avanço soma, recuo subtrai.
    if (estado_atual_encoder != estado_anterior_encoder) {
        distancia_total += i_sentido.load() * PASSO_ENCODER; // +0,5 (frente), -0,5 (ré) ou 0
    }

    // --- Velocidade por janela de tempo (metros percorridos / tempo decorrido) ---
    // Com recuo, distancia_total pode diminuir, então a velocidade fica negativa.
    janela.push_back({t0, distancia_total});
    while (janela.size() > 1 && (t0 - janela.front().first) > JANELA_DUR) {
        janela.pop_front();
    }

    double velocidade = 0.0;
    if (janela.size() >= 2) {
        double d_dist = janela.back().second - janela.front().second;
        double d_tempo = std::chrono::duration<double>(
            janela.back().first - janela.front().first).count();
        if (d_tempo > 1e-6) velocidade = d_dist / d_tempo;
    }
    velocidade_atual.store(velocidade);

    // Expõe a posição atual para a telemetria (lida pela bridge MQTT)
    posicao_x.store(distancia_total);

    // Envio para o Coletor
    buffer_distancia_coletor.push(distancia_total);

    estado_anterior_encoder = estado_atual_encoder;

    wcet_odometria.registrar_desde(t0);  // fim da medição (antes de reagendar)

    // Agendamento Assíncrono para o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(20));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_odometria(timer);
    });
}