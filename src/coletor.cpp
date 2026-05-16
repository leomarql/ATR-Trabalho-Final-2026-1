/*
coletor.cpp - Responsável por coletar os dados do Lidar e salvar no CSV
O que faz: Esta função é executada em uma thread separada e fica bloqueada esperando por novas leituras do LIDAR. 
Quando uma nova leitura chega, ela esvazia a fila de distâncias para pegar a posição mais recente, e então salva um 
registro no arquivo CSV com o timestamp, posição, leitura do teto e a confiança atual. 
O arquivo é aberto em modo append para garantir que os dados sejam preservados entre execuções e o cabeçalho é escrito apenas se o arquivo estiver vazio. 
*/

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <atomic>
#include "BufferCompartilhado.hpp"

extern BufferCompartilhado<int> buffer_lidar_coletor;
extern BufferCompartilhado<double> buffer_distancia_coletor;
extern std::atomic<int> confianca_atual; 
extern std::atomic<bool> executando;

void tarefa_coletor_dados() {
    std::ofstream arquivo_log;
    arquivo_log.open("log_inspecao.csv", std::ios::app);
    arquivo_log.seekp(0, std::ios::end); 
    if (arquivo_log.tellp() == 0) { 
        arquivo_log << "Timestamp_ms,Posicao_X_m,Posicao_Y_m,Leitura_Teto_cm,Confianca_%\n";
    }

    static double ultima_posicao_x = 0.0;

    while (executando.load()) {
        
        // Bloqueia esperando o Lidar bater o bumbo
        auto leitura_opt = buffer_lidar_coletor.pop();
        if (!leitura_opt.has_value()) break; 

        int leitura_teto = leitura_opt.value();

        // NOVO: Esvaziamento (Drain) blindado contra corridas de dados!
        // Tenta puxar até a fila esvaziar, sem bloquear a thread do coletor.
        while (true) {
            auto pos_opt = buffer_distancia_coletor.try_pop();
            if (!pos_opt.has_value()) {
                break; // Fila vazia, sai do loop de esvaziamento
            }
            ultima_posicao_x = pos_opt.value(); // Atualiza com o valor mais recente
        }

        int confianca = confianca_atual.load(); 

        auto agora = std::chrono::system_clock::now();
        auto tempo_ms = std::chrono::duration_cast<std::chrono::milliseconds>(agora.time_since_epoch()).count();

        arquivo_log << tempo_ms << ","
                    << std::fixed << std::setprecision(2) << ultima_posicao_x << ","
                    << std::fixed << std::setprecision(2) << 0.00 << ","
                    << leitura_teto << ","
                    << confianca << "\n";
                    
        arquivo_log.flush();
    }
}