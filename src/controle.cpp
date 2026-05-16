/*
Controle.cpp - Controlador PID para Navegação
O que faz: Roda assincronamente a cada 80ms.
Compara a velocidade que o robô deveria estar (j_sp_velocidade) com a velocidade calculada pela Odometria.
Aplica as equações matemática do Controlador PID (Proporcional, Integral e Derivativo) e gera um sinal PWM 
saturado (entre -100 e 100) para atuar nos motores via o_aceleracao.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <boost/asio.hpp>

// Variáveis Globais (Vêm do main.cpp)
extern std::atomic<double> j_sp_velocidade;
extern std::atomic<double> velocidade_atual;
extern std::atomic<int> o_aceleracao;

void callback_controle_navegacao(boost::asio::steady_timer& timer) {
    static double erro_integral = 0.0;
    static double erro_anterior = 0.0;
    
    const double Kp = 2.0, Ki = 0.5, Kd = 0.1;
    const double dt = 0.080; // 80ms

    // 1. Leitura do Setpoint e Variável de Processo
    double sp = j_sp_velocidade.load();
    double pv = velocidade_atual.load();

    // 2. Matemática do PID
    double erro = sp - pv;
    erro_integral += erro * dt;
    double derivada = (erro - erro_anterior) / dt;
    
    double saida = (Kp * erro) + (Ki * erro_integral) + (Kd * derivada);

    // 3. Saturação (Sinal de Atuação PWM)
    int saida_saturada = static_cast<int>(saida);
    if (saida_saturada > 100) saida_saturada = 100;
    if (saida_saturada < -100) saida_saturada = -100;

    // 4. Escrita na saída física
    o_aceleracao.store(saida_saturada);
    erro_anterior = erro;

    // Agendamento Assíncrono para o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(80));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_controle_navegacao(timer);
    });
}