/*
coletor.cpp - Responsável por coletar os dados do Lidar e salvar no CSV
O que faz: Esta função é executada em uma thread separada e fica bloqueada esperando por novas leituras do LIDAR.
Quando uma nova leitura chega, ela esvazia a fila de distâncias para pegar a posição mais recente, e então salva um
registro no arquivo CSV com o timestamp, posição (x,y), a confiança calculada ONLINE por densidade de medições e a
inclinação do piso medida pela IMU (EXTRA: túnel com declive).
O arquivo é aberto em modo append para garantir que os dados sejam preservados entre execuções e o cabeçalho é escrito apenas se o arquivo estiver vazio.
*/

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <deque>
#include <algorithm>
#include <cmath>
#include <utility>
#include "BufferCompartilhado.hpp"

extern BufferCompartilhado<int> buffer_lidar_coletor;
extern BufferCompartilhado<double> buffer_distancia_coletor;
extern std::atomic<int> confianca_atual; // Agora o coletor ESCREVE aqui (para telemetria da Etapa 2 ler)
extern std::atomic<double> i_imu_pitch;  // inclinação medida pela IMU (graus) - EXTRA: declive
extern std::atomic<bool> executando;

void tarefa_coletor_dados() {
    std::ofstream arquivo_log;
    arquivo_log.open("log_inspecao.csv", std::ios::app);
    arquivo_log.seekp(0, std::ios::end);
    if (arquivo_log.tellp() == 0) {
        arquivo_log << "Timestamp_ms,Posicao_X_m,Posicao_Y_cm,Confianca_%,Inclinacao_graus\n";
    }

    static double ultima_posicao_x = 0.0;

    // --- Histórico para cálculo de confiança por DENSIDADE de medições ---
    static std::deque<std::pair<double,double>> historico_pontos;
    const size_t MAX_HIST = 50;   // janela de pontos considerados
    const double RAIO_X = 1.5;    // metros
    const double RAIO_Y = 15.0;   // cm

    while (executando.load()) {

        // Bloqueia esperando o Lidar bater o bumbo
        auto leitura_opt = buffer_lidar_coletor.pop();
        if (!leitura_opt.has_value()) break;

        int leitura_teto = leitura_opt.value();

        // Esvaziamento (Drain) blindado contra corridas de dados.
        // Tenta puxar até a fila esvaziar, sem bloquear a thread do coletor.
        while (true) {
            auto pos_opt = buffer_distancia_coletor.try_pop();
            if (!pos_opt.has_value()) {
                break; // Fila vazia, sai do loop de esvaziamento
            }
            ultima_posicao_x = pos_opt.value(); // Atualiza com o valor mais recente
        }

        // --- Confiança por DENSIDADE de medições (online, no coletor) ---
        // Quanto mais pontos (x,y) já registrados perto do ponto atual, maior a confiança.
        double x_atual = ultima_posicao_x;
        double y_atual = static_cast<double>(leitura_teto);

        int vizinhos = 0;
        for (const auto& p : historico_pontos) {
            if (std::abs(p.first  - x_atual) <= RAIO_X &&
                std::abs(p.second - y_atual) <= RAIO_Y) {
                vizinhos++;
            }
        }

        int confianca = std::min(100, (vizinhos * 100) / static_cast<int>(MAX_HIST));

        historico_pontos.push_back({x_atual, y_atual});
        if (historico_pontos.size() > MAX_HIST) historico_pontos.pop_front();

        // Disponibiliza a confiança para a telemetria (MQTT) ler na Etapa 2
        confianca_atual.store(confianca);

        auto agora = std::chrono::system_clock::now();
        auto tempo_ms = std::chrono::duration_cast<std::chrono::milliseconds>(agora.time_since_epoch()).count();

        arquivo_log << tempo_ms << ","
                    << std::fixed << std::setprecision(2) << ultima_posicao_x << ","
                    << leitura_teto << ","
                    << confianca << ","
                    << std::fixed << std::setprecision(2) << i_imu_pitch.load() << "\n";

        arquivo_log.flush();
    }
}