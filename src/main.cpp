/*
main.cpp - Ponto de entrada do sistema de controle do robô autônomo.
O que faz: Este arquivo inicializa o sistema, configura os timers para as tarefas periódicas e gerencia o ciclo de vida das threads.
Ele também é responsável por um desligamento seguro, garantindo que todas as threads sejam notificadas para encerrar suas operações e
que os buffers compartilhados sejam fechados corretamente para evitar deadlocks.
Suporta dois modos via linha de comando:
  --offline (padrão): roda o mock interno (tarefa_mock_mundo) que simula sensores por 5s.
  --online          : NÃO roda o mock; aguarda o simulador físico externo (Python/MQTT) alimentar os sensores.
Ao final do programa, imprime as métricas de desempenho dos buffers (número de descartes) para análise.
*/

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <string>
#include <csignal>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"

// Variáveis Globais
std::atomic<bool> executando{true};
std::atomic<bool> e_automatico{true};
std::atomic<bool> i_encoder{false};
std::atomic<double> j_sp_velocidade{5.0};
std::atomic<double> velocidade_atual{0.0};
std::atomic<int> o_aceleracao{0};
std::atomic<int> i_lidar{200};
std::atomic<bool> e_inspecao{false};

std::atomic<int> confianca_atual{100};
std::atomic<int> limiar_anomalia{10};    // item 3 - configurável pela Operação Remota
std::atomic<bool> o_liga_camera{false};  // item 4 - atuador da câmera (Tabela 1)
std::mutex mtx_camera;
std::condition_variable cv_camera;

BufferCompartilhado<int> buffer_lidar_coletor(200);
BufferCompartilhado<double> buffer_distancia_coletor(200);

// Handler de sinal para encerrar o modo online com Ctrl+C
void handler_sinal(int) {
    executando.store(false);
}

// Declarações Externas
extern void callback_odometria(boost::asio::steady_timer& timer);
extern void callback_controle_navegacao(boost::asio::steady_timer& timer);
extern void callback_reconstrucao_teto(boost::asio::steady_timer& timer);
extern void callback_comando_navegacao(boost::asio::steady_timer& timer);
extern void tarefa_inspecao_camera();
extern void tarefa_coletor_dados();
extern void tarefa_mock_mundo();

int main(int argc, char* argv[]) {
    // --- Parsing dos argumentos de linha de comando ---
    bool modo_online = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--online") modo_online = true;
        // --offline é o padrão; aceito explicitamente por clareza
    }
    std::signal(SIGINT, handler_sinal);

    boost::asio::io_context io;

    boost::asio::steady_timer timer_odometria(io, std::chrono::milliseconds(20));
    boost::asio::steady_timer timer_controle(io, std::chrono::milliseconds(80));
    boost::asio::steady_timer timer_lidar(io, std::chrono::milliseconds(100));
    boost::asio::steady_timer timer_comando(io, std::chrono::milliseconds(200));

    timer_odometria.async_wait([&](const boost::system::error_code& e){ callback_odometria(timer_odometria); });
    timer_controle.async_wait([&](const boost::system::error_code& e){ callback_controle_navegacao(timer_controle); });
    timer_lidar.async_wait([&](const boost::system::error_code& e){ callback_reconstrucao_teto(timer_lidar); });
    timer_comando.async_wait([&](const boost::system::error_code& e){ callback_comando_navegacao(timer_comando); });

    // Instancia as Tarefas Consumidoras Puras
    std::thread t_camera(tarefa_inspecao_camera);
    std::thread t_coletor(tarefa_coletor_dados);

    // O mock só roda no modo offline (no online, o simulador externo alimenta os sensores)
    std::thread t_mundo;
    if (!modo_online) {
        t_mundo = std::thread(tarefa_mock_mundo);
    }

    // Threads do Produtor RT (Asio)
    std::vector<std::thread> thread_pool;
    for(int i = 0; i < 4; ++i) {
        thread_pool.emplace_back([&io](){ io.run(); });
    }

    if (!modo_online) {
        t_mundo.join(); // o mock dispara executando=false após 5s
    } else {
        std::cout << "--- MODO ONLINE: aguardando simulador/MQTT. Ctrl+C para encerrar. ---\n";
        while (executando.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // --- GRACEFUL SHUTDOWN ---
    std::cout << "--- DESLIGANDO O SISTEMA... ---\n";
    executando.store(false);
    io.stop();

    cv_camera.notify_all();

    // O jeito certo: invoca o close() nos buffers para destravar os consumidores
    buffer_lidar_coletor.close();
    buffer_distancia_coletor.close();

    if(t_camera.joinable()) t_camera.join();
    if(t_coletor.joinable()) t_coletor.join();

    for(auto& t : thread_pool) {
        if(t.joinable()) t.join();
    }

    // Imprime as métricas exigidas pelo revisor
    std::cout << "Metricas de Buffer:\n";
    std::cout << " - Descartes Lidar: " << buffer_lidar_coletor.get_descartes() << "\n";
    std::cout << " - Descartes Odometria: " << buffer_distancia_coletor.get_descartes() << "\n";
    std::cout << "--- SISTEMA ENCERRADO COM SEGURANCA ---\n";
    return 0;
}