/*
camera.cpp - Tarefa de Inspeção por Câmera
O que faz: Roda bloqueada esperando um gatilho do LIDAR (via Variável de Condição).
Quando acordada, simula um processamento pesado de imagem (multiplicação de matrizes O(N^3)) para representar a carga real de CPU exigida.
Após o processamento, libera a CPU e volta a dormir esperando o próximo gatilho.
*/

#include <iostream>
#include <vector>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

extern std::atomic<bool> e_inspecao;
extern std::atomic<bool> executando;
extern std::mutex mtx_camera;
extern std::condition_variable cv_camera;

void tarefa_inspecao_camera() {
    while(executando.load()) {
        std::unique_lock<std::mutex> lock(mtx_camera);
        
        // Dorme esperando o Lidar (ou o comando de desligar)
        cv_camera.wait(lock, []{ return e_inspecao.load() == true || !executando.load(); });
        
        if (!executando.load()) break;

        std::cout << "\n[CAMERA] Ativada! Iniciando processamento pesado de imagem...\n";
        
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
        
        std::cout << "[CAMERA] Imagem processada. CPU liberada.\n";
        e_inspecao.store(false); 
    }
}