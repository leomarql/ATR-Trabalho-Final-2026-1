#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "BufferCompartilhado.hpp"

// 1. DEFINIÇÃO DAS VARIÁVEIS GLOBAIS (Ocorre apenas aqui)
std::atomic<bool> e_inspecao{false};
std::atomic<int> i_lidar{200};
std::atomic<bool> o_liga_camera{false};
std::atomic<double> j_sp_velocidade{0.0};
std::atomic<double> velocidade_atual{0.0};
std::atomic<int> o_aceleracao{0};

// Nossos dois buffers oficiais
BufferCompartilhado<int> buffer_lidar_coletor(200);
BufferCompartilhado<double> buffer_distancia_coletor(200); 

std::mutex mtx_camera;
std::condition_variable cv_camera;

// 2. DECLARAÇÃO DAS FUNÇÕES EXTERNAS (Implementadas nos outros arquivos)
void tarefa_reconstrucao_teto();
void tarefa_controle_navegacao();

// 3. TAREFAS LOCAIS (Para o teste)
void tarefa_inspecao_camera() {
    while(true) {
        std::unique_lock<std::mutex> lock(mtx_camera);
        cv_camera.wait(lock, []() { return e_inspecao.load(); });

        std::cout << "[CAMERA] Acordei! Iniciando inspecao visual...\n";
        o_liga_camera.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        std::cout << "[CAMERA] Inspecao concluida. Voltando a dormir.\n\n";
        o_liga_camera.store(false);
        e_inspecao.store(false); 
    }
}

void tarefa_mock_mundo() {
    std::cout << "[MUNDO] Robo andando em trecho reto (Lidar 200cm)...\n"; 
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "[MUNDO] -> INJETANDO BURACO (Lidar 250cm)!\n";
    i_lidar.store(250); 
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "[MUNDO] -> Teto voltou ao normal (200cm)...\n";
    i_lidar.store(200); 
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

extern void tarefa_coletor_dados(); // Prototipo da função do coletor (Implementada em coletor.cpp)

int main() {
    std::cout << "--- INICIANDO SISTEMA MULTITAREFA ---\n\n";

    // Instancia todas as threads
    std::thread t_lidar(tarefa_reconstrucao_teto);
    std::thread t_controle(tarefa_controle_navegacao); // Adicionamos o PID aqui!
    std::thread t_camera(tarefa_inspecao_camera);
    std::thread t_mundo(tarefa_mock_mundo);
    std::thread t_coletor(tarefa_coletor_dados);
    
    // O mock (Mundo) dita o tempo de vida do programa no nosso teste
    t_mundo.join();

    // As threads infinitas rodam soltas em background 
    t_coletor.detach();
    t_lidar.detach();
    t_controle.detach();
    t_camera.detach();

    std::cout << "--- TESTE FINALIZADO ---\n";
    return 0;
}