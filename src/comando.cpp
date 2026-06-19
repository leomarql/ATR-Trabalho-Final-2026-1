/*
comando.cpp - Tarefa de Comando de Navegação
O que faz: Roda assincronamente a cada 200ms.
Decide o setpoint de velocidade (j_sp_velocidade) dependendo do modo de operação.

  - Modo AUTOMÁTICO: a lógica autônoma define o setpoint (velocidade de cruzeiro).
  - Modo MANUAL: NÃO sobrescreve o setpoint. Quem controla é o operador, via os
    comandos de direção (direita/esquerda/parar) recebidos pela ponte MQTT, que
    escrevem diretamente em j_sp_velocidade. Se esta tarefa zerasse o setpoint a
    cada ciclo, ela apagaria o comando do operador 200ms depois.

O setpoint é lido pelo controlador PID para gerar o sinal de atuação nos motores.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <boost/asio.hpp>

// Variáveis Globais (Nascem no main.cpp)
extern std::atomic<bool> e_automatico;
extern std::atomic<double> j_sp_velocidade;

void callback_comando_navegacao(boost::asio::steady_timer& timer) {
    // Modo automático: define a velocidade de cruzeiro autônoma.
    // Modo manual: não toca no setpoint (controlado pelos comandos de direção via MQTT).
    if (e_automatico.load() == true) {
        j_sp_velocidade.store(5.0);
    }

    // Agenda o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(200));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_comando_navegacao(timer);
    });
}