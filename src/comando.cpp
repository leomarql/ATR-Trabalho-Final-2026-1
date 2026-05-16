/*
comando.cpp - Tarefa de Comando de Navegação
O que faz: Roda assincronamente a cada 200ms.
Decide o setpoint de velocidade (j_sp_velocidade) dependendo do modo de operação (autônomo ou manual). 
O modo pode ser alternado a qualquer momento, e o comando de navegação se adapta imediatamente. 
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
    // Define o setpoint dependendo do modo
    if (e_automatico.load() == true) {
        j_sp_velocidade.store(5.0); 
    } else {
        j_sp_velocidade.store(0.0);
    }

    // Agenda o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(200));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_comando_navegacao(timer);
    });
}