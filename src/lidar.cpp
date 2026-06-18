/*
lidar.cpp - Implementação do módulo de processamento de dados do LIDAR.
O que faz: Este módulo é responsável por ler os dados do LIDAR, processá-los para detectar buracos e enviar as informações relevantes para a câmera.
Ele utiliza uma média móvel para suavizar as leituras, calcula a variância para estimar a confiança da detecção e implementa uma lógica de borda para 
acordar a câmera apenas quando um buraco é detectado pela primeira vez.
Além disso, os dados processados são enviados para um buffer compartilhado, permitindo que a câmera acesse as informações de forma thread-safe. 
O módulo é projetado para operar em um ambiente multithread, garantindo a segurança e eficiência na comunicação entre os componentes do sistema. 
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"

extern std::atomic<int> i_lidar;
extern std::atomic<bool> e_inspecao;
extern BufferCompartilhado<int> buffer_lidar_coletor;
extern std::mutex mtx_camera;
extern std::condition_variable cv_camera;
extern std::atomic<int> confianca_atual;
extern std::atomic<int> limiar_anomalia;
extern std::atomic<bool> o_liga_camera;

void callback_reconstrucao_teto(boost::asio::steady_timer& timer) {
    static int historico[5] = {200, 200, 200, 200, 200};
    static int indice = 0;
    
    // NOVIDADE: Variável para rastrear se já estamos dentro de um buraco
    static bool buraco_anterior = false; 

    int leitura_atual = i_lidar.load();

    // 1. Média Móvel
    historico[indice] = leitura_atual;
    indice = (indice + 1) % 5;
    
    int soma = 0;
    for(int i = 0; i < 5; i++) soma += historico[i];
    int media_movel = soma / 5;

    // 3. Detecção de Anomalia 
    bool buraco_atual = std::abs(leitura_atual - media_movel) > limiar_anomalia.load();

    // LÓGICA DE BORDA: Só acorda a câmera na transição de Falso -> Verdadeiro
    if (buraco_atual == true && buraco_anterior == false) {
        e_inspecao.store(true);
        o_liga_camera.store(true); // Aciona a câmera para começar a inspecionar
        cv_camera.notify_one(); // Grita APENAS UMA VEZ no início do buraco
    } 
    // Quando o robô sair do buraco, abaixa a flag para estar pronto para o próximo
    else if (buraco_atual == false && buraco_anterior == true) {
        e_inspecao.store(false);
        o_liga_camera.store(false); // Desliga a câmera
    }
    
    buraco_anterior = buraco_atual; // Salva o estado para o próximo ciclo (100ms)

    // 4. Envio de dados
    buffer_lidar_coletor.push(media_movel);

    // Agendamento Assíncrono
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(100));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_reconstrucao_teto(timer);
    });
}