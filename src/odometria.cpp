/* 
Odometria.cpp - Cálculo de Distância e Velocidade a partir do Encoder
O que faz: Roda assincronamente a cada 20ms. 
Lê o sensor do encoder detectando a borda de subida (transição de falso para verdadeiro). 
A cada pulso, incrementa a distância percorrida, calcula a velocidade cinemática atual 
e abastece o controlador PID e o Coletor de Dados.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"

// Variáveis Globais (Vêm do main.cpp)
extern std::atomic<bool> i_encoder;
extern std::atomic<double> velocidade_atual;
extern BufferCompartilhado<double> buffer_distancia_coletor;

void callback_odometria(boost::asio::steady_timer& timer) {
    // Variáveis estáticas mantêm o valor entre os ciclos
    static bool estado_anterior_encoder = false;
    static double distancia_total = 0.0;
    static double distancia_anterior = 0.0;
    const double dt = 0.020; // 20ms

    bool estado_atual_encoder = i_encoder.load();

    // Detecção de Borda de Subida (0 -> 1)
    if (estado_anterior_encoder == false && estado_atual_encoder == true) {
        distancia_total += 1.0; // Andou 1 metro
    }

    // Cálculo da Velocidade Instantânea
    double velocidade = (distancia_total - distancia_anterior) / dt;
    velocidade_atual.store(velocidade);
    
    // Envio para o Coletor
    buffer_distancia_coletor.push(distancia_total);

    estado_anterior_encoder = estado_atual_encoder;
    distancia_anterior = distancia_total;

    // Agendamento Assíncrono para o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(20));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_odometria(timer);
    });
}