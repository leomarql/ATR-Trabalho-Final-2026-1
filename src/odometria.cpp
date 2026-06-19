/* 
Odometria.cpp - Cálculo de Distância e Velocidade a partir do Encoder
O que faz: Roda assincronamente a cada 20ms. 
Lê o sensor do encoder e, conforme o enunciado, conta 1 metro percorrido a CADA
troca de estado do encoder (0->1 ou 1->0). A cada metro, atualiza a distância,
calcula a velocidade cinemática atual, abastece o controlador PID e o Coletor,
e expõe a posição X (posicao_x) para a telemetria MQTT.
Mede o próprio tempo de execução (WCET) para a análise de escalonabilidade.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"
#include "Profiler.hpp"

// Variáveis Globais (Vêm do main.cpp)
extern std::atomic<bool> i_encoder;
extern std::atomic<double> velocidade_atual;
extern std::atomic<double> posicao_x;   // exposta para a telemetria
extern BufferCompartilhado<double> buffer_distancia_coletor;
extern MedidorWCET wcet_odometria;      // medidor de tempo de execução

void callback_odometria(boost::asio::steady_timer& timer) {
    auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET

    // Variáveis estáticas mantêm o valor entre os ciclos
    static bool estado_anterior_encoder = false;
    static double distancia_total = 0.0;
    static double distancia_anterior = 0.0;
    const double dt = 0.020; // 20ms

    bool estado_atual_encoder = i_encoder.load();

    // Conta 1 metro a cada TROCA DE ESTADO (subida ou descida), conforme o enunciado:
    // "o encoder gera uma troca de estado (0->1 ou 1->0) a cada metro percorrido".
    if (estado_atual_encoder != estado_anterior_encoder) {
        distancia_total += 1.0; // Andou 1 metro
    }

    // Cálculo da Velocidade Instantânea
    double velocidade = (distancia_total - distancia_anterior) / dt;
    velocidade_atual.store(velocidade);

    // Expõe a posição atual para a telemetria (lida pela bridge MQTT)
    posicao_x.store(distancia_total);

    // Envio para o Coletor
    buffer_distancia_coletor.push(distancia_total);

    estado_anterior_encoder = estado_atual_encoder;
    distancia_anterior = distancia_total;

    wcet_odometria.registrar_desde(t0);  // fim da medição (antes de reagendar)

    // Agendamento Assíncrono para o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(20));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_odometria(timer);
    });
}