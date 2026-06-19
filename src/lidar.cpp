/*
lidar.cpp - Implementação do módulo de processamento de dados do LIDAR.
O que faz: Este módulo é responsável por ler os dados do LIDAR, processá-los para detectar buracos e enviar as informações relevantes para a câmera.
Ele utiliza uma média móvel para suavizar as leituras e implementa uma lógica de borda para acordar a câmera apenas quando um buraco é detectado pela primeira vez.
O limiar de detecção de anomalia é configurável pela Operação Remota (variável atômica limiar_anomalia).
Ao detectar uma anomalia, aciona o atuador da câmera (o_liga_camera) e sinaliza a tarefa de inspeção.
Os dados processados são enviados para um buffer compartilhado, permitindo que o coletor acesse as informações de forma thread-safe.
Mede o próprio tempo de execução (WCET) para a análise de escalonabilidade.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"
#include "Profiler.hpp"

extern std::atomic<int> i_lidar;
extern std::atomic<bool> e_inspecao;
extern BufferCompartilhado<int> buffer_lidar_coletor;
extern std::mutex mtx_camera;
extern std::condition_variable cv_camera;
extern std::atomic<int> limiar_anomalia;   // configurável pela Operação Remota (Etapa 2)
extern std::atomic<bool> o_liga_camera;     // atuador da câmera (Tabela 1)
extern MedidorWCET wcet_lidar;              // medidor de tempo de execução

void callback_reconstrucao_teto(boost::asio::steady_timer& timer) {
    auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET

    static int historico[5] = {200, 200, 200, 200, 200};
    static int indice = 0;

    // Rastreia se já estamos dentro de um buraco (lógica de borda)
    static bool buraco_anterior = false;

    int leitura_atual = i_lidar.load();

    // 1. Média Móvel
    historico[indice] = leitura_atual;
    indice = (indice + 1) % 5;

    int soma = 0;
    for(int i = 0; i < 5; i++) soma += historico[i];
    int media_movel = soma / 5;

    // 2. Detecção de Anomalia (limiar configurável pela Operação Remota)
    bool buraco_atual = std::abs(leitura_atual - media_movel) > limiar_anomalia.load();

    // LÓGICA DE BORDA: Só acorda a câmera na transição de Falso -> Verdadeiro
    if (buraco_atual == true && buraco_anterior == false) {
        e_inspecao.store(true);
        o_liga_camera.store(true);    // liga o atuador da câmera
        cv_camera.notify_one();       // grita APENAS UMA VEZ no início do buraco
    }
    // Quando o robô sair do buraco, abaixa as flags para estar pronto para o próximo
    else if (buraco_atual == false && buraco_anterior == true) {
        e_inspecao.store(false);
        o_liga_camera.store(false);   // desliga o atuador da câmera
    }

    buraco_anterior = buraco_atual; // Salva o estado para o próximo ciclo (100ms)

    // 3. Envio de dados
    buffer_lidar_coletor.push(media_movel);

    wcet_lidar.registrar_desde(t0);  // fim da medição (antes de reagendar)

    // Agendamento Assíncrono
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(100));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_reconstrucao_teto(timer);
    });
}