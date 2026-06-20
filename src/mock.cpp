/*
mock.cpp - Mock de sensores do MODO OFFLINE (--offline).
O que faz: Roda em uma thread separada e simula o "mundo físico" de forma mínima,
SEM depender do simulador Python nem do MQTT. É selecionado pela flag --offline
(padrão) e serve para testar o núcleo C++ isoladamente.
Injeta pulsos no encoder a cada 100ms e simula um buraco no teto (leitura do LIDAR)
aos 2 segundos, restaurando o teto aos 3 segundos. Após 5 segundos, dispara a flag
de desligamento para encerrar o sistema.
No modo --online esta tarefa NÃO roda: os sensores passam a vir do simulador físico
via ponte MQTT (ver src/mqtt_bridge.cpp e python/simulador.py).
*/

#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>

extern std::atomic<bool> i_encoder;
extern std::atomic<int> i_lidar;
extern std::atomic<bool> executando;

void tarefa_mock_mundo() {
    std::cout << "--- SIMULACAO INICIADA (5 SEGUNDOS) ---\n";
    for (int i = 0; i < 50; ++i) { 
        i_encoder.store(!i_encoder.load()); // Simula os pulsos do encoder

        if (i == 20) i_lidar.store(280); // Injeta o buraco
        else if (i == 30) i_lidar.store(200); // Restaura o teto

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Finaliza o teste
    executando.store(false);
}