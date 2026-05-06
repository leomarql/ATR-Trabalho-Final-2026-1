#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include "BufferCompartilhado.hpp"

extern std::atomic<bool> i_encoder;
extern std::atomic<double> velocidade_atual;
extern BufferCompartilhado<double> buffer_distancia_coletor; // Note: alterado para double para precisão

void tarefa_odometria() {
    const auto periodo = std::chrono::milliseconds(20); // Loop estritamente a cada 20ms
    auto proximo_ciclo = std::chrono::steady_clock::now() + periodo;
    
    bool estado_anterior = false;
    double distancia_total = 0.0;
    
    // Para o cálculo da velocidade v = delta_s / delta_t
    // delta_t é o período do loop (0.02s)
    const double dt = 0.02; 

    while(true) {
        bool estado_atual = i_encoder.load();
        double delta_s = 0.0;

        // 1 & 2. Detecção de Borda (0 -> 1 ou false -> true)
        // O encoder gera troca de estado a cada metro [cite: 89, 119]
        if (estado_anterior == false && estado_atual == true) {
            delta_s = 1.0; // Andou 1 metro
            distancia_total += delta_s;
            // std::cout << "[ODOMETRIA] +1 metro detectado! Total: " << distancia_total << "m\n";
        }
        estado_anterior = estado_atual;

        // 3. Matemática da Velocidade (v = ds / dt)
        // Salva na variável global para o PID usar 
        double v = delta_s / dt;
        velocidade_atual.store(v);

        // 4. Envio de Dados
        // Envia a distância acumulada para o buffer do coletor [cite: 90, 94]
        buffer_distancia_coletor.push(distancia_total);

        // Mantém o determinismo temporal
        std::this_thread::sleep_until(proximo_ciclo);
        proximo_ciclo += periodo;
    }
}