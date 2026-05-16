/*
main.cpp - Ponto de entrada do sistema de controle do robô autônomo.
O que faz: Este arquivo inicializa o sistema, configura os timers para as tarefas periódicas e gerencia o ciclo de vida das threads. 
Ele também é responsável por um desligamento seguro, garantindo que todas as threads sejam notificadas para encerrar suas operações e 
que os buffers compartilhados sejam fechados corretamente para evitar deadlocks. 
Além disso, ao final do programa, ele imprime as métricas de desempenho dos buffers, como o número de descartes ocorridos, para análise posterior. 
*/

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
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
std::mutex mtx_camera;
std::condition_variable cv_camera;

BufferCompartilhado<int> buffer_lidar_coletor(200);
BufferCompartilhado<double> buffer_distancia_coletor(200);

// Declarações Externas
extern void callback_odometria(boost::asio::steady_timer& timer);
extern void callback_controle_navegacao(boost::asio::steady_timer& timer);
extern void callback_reconstrucao_teto(boost::asio::steady_timer& timer);
extern void callback_comando_navegacao(boost::asio::steady_timer& timer);
extern void tarefa_inspecao_camera();
extern void tarefa_coletor_dados(); // Nova declaração da thread
extern void tarefa_mock_mundo();

int main() {
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
    std::thread t_mundo(tarefa_mock_mundo);

    // Threads do Produtor RT (Asio)
    std::vector<std::thread> thread_pool;
    for(int i = 0; i < 4; ++i) {
        thread_pool.emplace_back([&io](){ io.run(); });
    }

    t_mundo.join(); 

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