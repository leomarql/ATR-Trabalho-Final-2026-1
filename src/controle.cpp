/*
Controle.cpp - Controlador PID para Navegação (com compensação de declive)
O que faz: Roda assincronamente a cada 80ms.
Compara a velocidade que o robô deveria estar (j_sp_velocidade) com a velocidade calculada pela Odometria.
Aplica as equações do Controlador PID (Proporcional, Integral e Derivativo) e gera um sinal PWM 
saturado (entre -100 e 100) para atuar nos motores via o_aceleracao.

Recursos de robustez:
  - ANTI-WINDUP: o termo integral é limitado para não acumular indefinidamente
    (o que antes mantinha o robô acelerando mesmo após o setpoint cair).
  - FREIO: quando freio_ativo é levantado (comando "parar"), corta o atuador e
    reseta o estado do PID, garantindo parada firme e determinística.
  - FEED-FORWARD DE DECLIVE (EXTRA / IMU): usa a inclinação medida pela IMU
    (i_imu_pitch) para somar, de forma proativa, o esforço de motor que cancela a
    componente da gravidade na rampa. Assim o robô mantém a velocidade na subida e
    na descida sem depender do integral "perceber" o erro depois — exatamente o
    propósito do IMU em túneis com declive.

Mede o próprio tempo de execução (WCET) para a análise de escalonabilidade.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <cmath>
#include <boost/asio.hpp>
#include "Profiler.hpp"

// Variáveis Globais (Vêm do main.cpp)
extern std::atomic<double> j_sp_velocidade;
extern std::atomic<double> velocidade_atual;
extern std::atomic<int> o_aceleracao;
extern std::atomic<bool> freio_ativo;     // parada firme (comando "parar")
extern std::atomic<double> i_imu_pitch;   // inclinação medida pela IMU (graus)
extern MedidorWCET wcet_controle;         // medidor de tempo de execução

void callback_controle_navegacao(boost::asio::steady_timer& timer) {
    auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET

    static double erro_integral = 0.0;
    static double erro_anterior = 0.0;

    const double Kp = 2.0, Ki = 0.5, Kd = 0.1;
    const double dt = 0.080; // 80ms
    const double LIM_INTEGRAL = 200.0; // anti-windup: Ki*200 = 100 (limite do atuador)
    const double PI = 3.14159265358979323846;
    // Ganho de feed-forward de declive: calibrado a ~100*g/ACEL_MAX (≈ 245) para que
    // KFF*sin(pitch) [%] cancele a componente da gravidade g*sin(pitch) [m/s²] na rampa.
    const double KFF_INCLINACAO = 245.0;

    int saida_saturada;

    if (freio_ativo.load()) {
        // FREIO ATIVO: corta o atuador e zera o estado do PID.
        // Isso evita que o termo integral acumulado continue empurrando o robô
        // e garante uma parada firme assim que o operador comanda "parar".
        erro_integral = 0.0;
        erro_anterior = 0.0;
        saida_saturada = 0;
    } else {
        // 1. Leitura do Setpoint e Variável de Processo
        double sp = j_sp_velocidade.load();
        double pv = velocidade_atual.load();

        // 2. Matemática do PID
        double erro = sp - pv;
        erro_integral += erro * dt;

        // Anti-windup: limita o termo integral para não saturar indefinidamente
        if (erro_integral >  LIM_INTEGRAL) erro_integral =  LIM_INTEGRAL;
        if (erro_integral < -LIM_INTEGRAL) erro_integral = -LIM_INTEGRAL;

        double derivada = (erro - erro_anterior) / dt;

        // Feed-forward de declive: compensa proativamente a gravidade na rampa
        // usando a inclinação lida pela IMU (subida -> soma esforço; descida -> reduz).
        double pitch_rad = i_imu_pitch.load() * PI / 180.0;
        double ff_inclinacao = KFF_INCLINACAO * std::sin(pitch_rad);

        double saida = (Kp * erro) + (Ki * erro_integral) + (Kd * derivada) + ff_inclinacao;

        // 3. Saturação (Sinal de Atuação PWM)
        saida_saturada = static_cast<int>(saida);
        if (saida_saturada > 100) saida_saturada = 100;
        if (saida_saturada < -100) saida_saturada = -100;

        erro_anterior = erro;
    }

    // 4. Escrita na saída física
    o_aceleracao.store(saida_saturada);

    wcet_controle.registrar_desde(t0);  // fim da medição (antes de reagendar)

    // Agendamento Assíncrono para o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(80));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_controle_navegacao(timer);
    });
}