#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "BufferCompartilhado.hpp"

// 1. declaração extern: Puxando os buffers que vão nascer no main
extern BufferCompartilhado<int> buffer_lidar_coletor;
extern BufferCompartilhado<double> buffer_distancia_coletor;

void tarefa_coletor_dados() {
    // 2. Abertura do Arquivo de Log no modo "Append" (adiciona ao final)
    // Usamos arquivo local na pasta de execução
    std::ofstream arquivo_log("log_inspecao.csv", std::ios::app);

    // Se o arquivo abriu corretamente, escrevemos o cabeçalho
    if (arquivo_log.is_open()) {
        arquivo_log << "Timestamp_ms,Posicao_X_m,Leitura_Teto_cm,Confianca_%\n";
    } else {
        std::cerr << "[COLETOR] Erro critico: Nao foi possivel criar log_inspecao.csv\n";
        return; // Mata a thread se não puder gravar
    }

    double posicao_atual_x = 0.0;

    while(true) {
        // 3. Ponto chave: A thread dorme aqui 
        // Ela só acorda quando o Lidar colocar um dado novo (a cada 100ms)
        int leitura_teto = buffer_lidar_coletor.pop();

        // 4. Sincronização de Frequências (Lidar vs Odometria)
        // Esvazia o buffer de distância para pegar sempre a posição MAIS RECENTE
        int amostras_posicao = 0;
        while(buffer_distancia_coletor.size() > 0) {
            posicao_atual_x = buffer_distancia_coletor.pop();
            amostras_posicao++;
        }

        // 5. Cálculo do Nível de Confiança (Regra exigida no PDF)
        // "Quanto mais medições próximas, maior o nível de confiança"
        // Se a odometria mandou várias amostras no intervalo, a confiança é alta.
        int confianca = 100;
        if (amostras_posicao < 3) confianca = 50; 
        if (amostras_posicao == 0) confianca = 10; 

        // 6. Geração do Timestamp preciso
        auto agora = std::chrono::system_clock::now();
        auto tempo_ms = std::chrono::duration_cast<std::chrono::milliseconds>(agora.time_since_epoch()).count();

        // 7. Gravação em Disco (I/O)
        arquivo_log << tempo_ms << ","
                    << std::fixed << std::setprecision(2) << posicao_atual_x << ","
                    << leitura_teto << ","
                    << confianca << "\n";

        // Força a gravação imediata no disco (evita perda de dados se o robô desligar do nada)
        arquivo_log.flush();
    }
}