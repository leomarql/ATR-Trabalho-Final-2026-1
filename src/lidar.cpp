#include <iostream>
#include <queue>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "BufferCompartilhado.hpp"

// DECLARAÇÃO EXTERN: Avisa que essas variáveis existem em outro arquivo
extern std::atomic<bool> e_inspecao;
extern std::atomic<int> i_lidar;
extern BufferCompartilhado<int> buffer_lidar_coletor;
extern std::mutex mtx_camera;
extern std::condition_variable cv_camera;

void tarefa_reconstrucao_teto() {
    const auto periodo = std::chrono::milliseconds(100);
    auto proximo_ciclo = std::chrono::steady_clock::now() + periodo;
    const int TAMANHO_JANELA = 5;
    std::queue<int> janela_filtro;
    int soma_janela = 0;
    int media_anterior = -1;
    const int LIMIAR_FALHA = 15;

    while(true) {
        int leitura_crua = i_lidar.load();
        janela_filtro.push(leitura_crua);
        soma_janela += leitura_crua;

        if (janela_filtro.size() > TAMANHO_JANELA) {
            soma_janela -= janela_filtro.front();
            janela_filtro.pop();
        }

        int media_atual = soma_janela / janela_filtro.size();
        buffer_lidar_coletor.push(media_atual);

        if (media_anterior != -1) {
            int variacao = std::abs(media_atual - media_anterior);
            if (variacao > LIMIAR_FALHA && !e_inspecao.load()) {
                std::cout << "\n[LIDAR] ANOMALIA DETECTADA! Variacao: " << variacao << "cm\n";
                std::cout << "[LIDAR] Disparando cv.notify_one() para acordar a Camera...\n";
                e_inspecao.store(true);
                cv_camera.notify_one(); 
            }
        }
        media_anterior = media_atual;
        std::this_thread::sleep_until(proximo_ciclo);
        proximo_ciclo += periodo;
    }
}