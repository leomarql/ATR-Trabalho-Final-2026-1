/*
camera.cpp - Tarefa de Inspeção por Câmera
O que faz: Roda bloqueada esperando um gatilho do LIDAR (via Variável de Condição).
Quando acordada, simula um processamento pesado de imagem (multiplicação de matrizes O(N^3)) para representar a carga real de CPU exigida.
Após o processamento, libera a CPU e volta a dormir esperando o próximo gatilho.
Mede o tempo de execução do processamento pesado (WCET) para a análise de
escalonabilidade. OBS: esta é uma tarefa ORIENTADA A EVENTO (não periódica), em
thread nativa dedicada — por isso não entra no conjunto periódico Rate Monotonic.
*/

#include <iostream>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "Profiler.hpp"

extern std::atomic<bool> gatilho_camera;  // gatilho ÚNICO disparado pelo lidar na entrada da anomalia
extern std::atomic<bool> executando;
extern std::mutex mtx_camera;
extern std::condition_variable cv_camera;
extern MedidorWCET wcet_camera;   // medidor de tempo de execução

void tarefa_inspecao_camera() {
    while(executando.load()) {
        std::unique_lock<std::mutex> lock(mtx_camera);
        
        // Dorme esperando o GATILHO do Lidar (ou o comando de desligar).
        // O gatilho é único por anomalia, então a câmera roda o processamento
        // pesado UMA vez por falha, sem enfileirar trabalho a cada ciclo do lidar.
        cv_camera.wait(lock, []{ return gatilho_camera.load() == true || !executando.load(); });
        
        if (!executando.load()) break;

        std::cout << "\n[CAMERA] Ativada! Iniciando processamento pesado de imagem...\n";

        auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET

        // Carga Real de CPU (Multiplicação de Matrizes O(N^3))
        const int SIZE = 400; 
        std::vector<std::vector<double>> A(SIZE, std::vector<double>(SIZE, 1.001));
        std::vector<std::vector<double>> B(SIZE, std::vector<double>(SIZE, 1.002));
        std::vector<std::vector<double>> C(SIZE, std::vector<double>(SIZE, 0.0));
        
        for(int i = 0; i < SIZE; ++i) {
            for(int j = 0; j < SIZE; ++j) {
                for(int k = 0; k < SIZE; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        wcet_camera.registrar_desde(t0);  // fim da medição do processamento pesado

        std::cout << "[CAMERA] Imagem processada. CPU liberada.\n";
        gatilho_camera.store(false);  // consome o gatilho; e_inspecao é controlado pelo lidar (região)
    }
}