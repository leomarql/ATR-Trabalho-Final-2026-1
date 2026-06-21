/*
comando.cpp - Tarefa de Comando de Navegação
O que faz: Roda assincronamente a cada 200ms.
Decide o setpoint de velocidade (j_sp_velocidade) conforme o modo de operação.

  - Modo AUTOMÁTICO: a lógica autônoma define o setpoint na velocidade de cruzeiro.
    O "andar mais devagar" durante a inspeção de uma anomalia (Tarefa 4 do enunciado)
    NÃO é feito aqui: é o controle (controle.cpp) que limita a velocidade enquanto
    e_inspecao está ativo, de modo que a redução valha tanto no automático quanto no
    manual (estado e_inspecao da Tabela 2: "navegando com velocidade limitada").
  - Modo MANUAL: NÃO sobrescreve o setpoint. Quem controla é o operador, via os
    comandos de direção (direita/esquerda/parar) recebidos pela ponte MQTT, que
    escrevem diretamente em j_sp_velocidade. Se esta tarefa zerasse o setpoint a
    cada ciclo, ela apagaria o comando do operador 200ms depois.

O setpoint é lido pelo controlador PID para gerar o sinal de atuação nos motores.
Mede o próprio tempo de execução (WCET) para a análise de escalonabilidade.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <boost/asio.hpp>
#include "Profiler.hpp"

// Variáveis Globais (Nascem no main.cpp)
extern std::atomic<bool> e_automatico;
extern std::atomic<double> j_sp_velocidade;
extern MedidorWCET wcet_comando;            // medidor de tempo de execução

// Velocidade de cruzeiro da navegação autônoma (m/s).
// A redução durante a inspeção de anomalias é aplicada no controle (controle.cpp),
// que limita a velocidade em qualquer modo enquanto e_inspecao está ativo.
// Ajustada para 3,5 m/s: a uma cruzeiro mais alta o robô atravessava as anomalias
// (2-4 m) antes de terminar de desacelerar, tornando a redução quase imperceptível.
// A 3,5 m/s a anomalia dura mais que o tempo de frenagem e a queda para a velocidade
// de inspeção (2 m/s) fica visível, sem afetar PID, escalonabilidade ou demais módulos.
static constexpr double VELOCIDADE_CRUZEIRO = 3.5;

void callback_comando_navegacao(boost::asio::steady_timer& timer) {
    auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET

    // Modo AUTOMÁTICO: define a velocidade de cruzeiro. O "andar mais devagar" na
    // inspeção (Tarefa 4) é feito pelo controle, que limita a velocidade quando
    // e_inspecao está ativo — assim vale tanto no automático quanto no manual.
    // Modo MANUAL: NÃO toca no setpoint (controlado pelos comandos de direção via MQTT).
    if (e_automatico.load() == true) {
        j_sp_velocidade.store(VELOCIDADE_CRUZEIRO);
    }

    wcet_comando.registrar_desde(t0);  // fim da medição (antes de reagendar)

    // Agenda o próximo ciclo
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(200));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_comando_navegacao(timer);
    });
}