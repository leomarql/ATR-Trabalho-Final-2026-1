/*
main.cpp - Ponto de entrada do sistema de controle do robô autônomo.
O que faz: Este arquivo inicializa o sistema, configura os timers para as tarefas periódicas e gerencia o ciclo de vida das threads. 
Ele também é responsável por um desligamento seguro, garantindo que todas as threads sejam notificadas para encerrar suas operações e 
que os buffers compartilhados sejam fechados corretamente para evitar deadlocks. 
Suporta dois modos via linha de comando:
  --offline (padrão): roda o mock interno (tarefa_mock_mundo) que simula sensores por 5s.
  --online          : NÃO roda o mock; sobe a ponte MQTT e aguarda o simulador físico externo.
Ao final do programa, imprime as métricas dos buffers (descartes) e os WCET medidos das tarefas.
*/

#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include <string>
#include <csignal>
#include "BufferCompartilhado.hpp"
#include "Profiler.hpp"

// Variáveis Globais
std::atomic<bool> executando{true};
std::atomic<bool> e_automatico{true};
std::atomic<bool> i_encoder{false};
std::atomic<int> i_sentido{1};            // sentido do movimento: +1 avanço, -1 recuo, 0 parado
std::atomic<double> i_imu_pitch{0.0};     // inclinação medida pela IMU em graus (EXTRA: declive)
std::atomic<double> j_sp_velocidade{5.0}; 
std::atomic<double> velocidade_atual{0.0};
std::atomic<int> o_aceleracao{0};
std::atomic<int> i_lidar{200}; 
std::atomic<bool> e_inspecao{false};
std::atomic<int> limiar_anomalia{10};    // configurável pela Operação Remota
std::atomic<bool> o_liga_camera{false};  // atuador da câmera (Tabela 1)
std::atomic<double> posicao_x{0.0};      // posição X exposta pela odometria p/ telemetria
std::atomic<bool> freio_ativo{false};    // parada firme (comando "parar" da Operação Remota)

std::atomic<int> confianca_atual{100};
std::mutex mtx_camera;
std::condition_variable cv_camera;

BufferCompartilhado<int> buffer_lidar_coletor(200);
BufferCompartilhado<double> buffer_distancia_coletor(200);

// Medidores de WCET (um por tarefa). Lidos pelas tarefas via 'extern'.
MedidorWCET wcet_odometria("Odometria (T=20ms) ");
MedidorWCET wcet_controle ("Controle  (T=80ms) ");
MedidorWCET wcet_lidar    ("Lidar     (T=100ms)");
MedidorWCET wcet_comando  ("Comando   (T=200ms)");
MedidorWCET wcet_camera   ("Camera    (evento)  ");

// Declarações Externas
extern void callback_odometria(boost::asio::steady_timer& timer);
extern void callback_controle_navegacao(boost::asio::steady_timer& timer);
extern void callback_reconstrucao_teto(boost::asio::steady_timer& timer);
extern void callback_comando_navegacao(boost::asio::steady_timer& timer);
extern void tarefa_inspecao_camera();
extern void tarefa_coletor_dados();
extern void tarefa_mock_mundo();
extern void tarefa_mqtt_bridge();   // ponte MQTT (thread dedicada de rede)

void handler_sinal(int) { // handler de sinal para o modo online encerrar com Ctrl+C
    executando.store(false);
}

int main(int argc, char* argv[]) {
    bool modo_online = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--online") modo_online = true;
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

    // Threads do Consumidor 
    std::thread t_camera(tarefa_inspecao_camera);
    std::thread t_coletor(tarefa_coletor_dados);

    // Modo offline: mock interno gera sensores. Modo online: ponte MQTT + simulador externo.
    std::thread t_mundo;
    std::thread t_mqtt;
    if (!modo_online) {
        t_mundo = std::thread(tarefa_mock_mundo);
    } else {
        t_mqtt = std::thread(tarefa_mqtt_bridge);
    }

    // Threads do Produtor RT (Asio)
    std::vector<std::thread> thread_pool;
    for(int i = 0; i < 4; ++i) {
        thread_pool.emplace_back([&io](){ io.run(); });
    }

    if (!modo_online) { // Modo Offline: roda o mundo simulado e depois encerra
        t_mundo.join();
    } else { // Modo Online: espera o usuário encerrar com Ctrl+C, mantendo o sistema rodando
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
    if(t_mqtt.joinable()) t_mqtt.join();

    for(auto& t : thread_pool) {
        if(t.joinable()) t.join();
    }

    // Imprime as métricas de buffer
    std::cout << "Metricas de Buffer:\n";
    std::cout << " - Descartes Lidar: " << buffer_lidar_coletor.get_descartes() << "\n";
    std::cout << " - Descartes Odometria: " << buffer_distancia_coletor.get_descartes() << "\n";

    // Imprime os WCET medidos (base para a análise de escalonabilidade)
    std::cout << "\nAnalise de WCET (tempos de execucao medidos):\n";
    auto imprimir = [](const MedidorWCET& m){
        std::cout << std::fixed << std::setprecision(1)
                  << " - " << m.nome()
                  << " | WCET = " << m.wcet_us()  << " us"
                  << " | media = " << m.media_us() << " us"
                  << " | amostras = " << m.amostras() << "\n";
    };
    imprimir(wcet_odometria);
    imprimir(wcet_controle);
    imprimir(wcet_lidar);
    imprimir(wcet_comando);
    imprimir(wcet_camera);

    std::cout << "--- SISTEMA ENCERRADO COM SEGURANCA ---\n";
    return 0;
}