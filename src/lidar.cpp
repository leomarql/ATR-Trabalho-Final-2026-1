/*
lidar.cpp - Implementação do módulo de processamento de dados do LIDAR.
O que faz: Este módulo é responsável por ler os dados do LIDAR, processá-los para detectar buracos e enviar as informações relevantes para a câmera.
Ele utiliza uma média móvel para suavizar as leituras e implementa uma lógica de borda para acordar a câmera apenas quando um buraco é detectado pela primeira vez.
O limiar de detecção de anomalia é configurável pela Operação Remota (variável atômica limiar_anomalia).
Ao detectar uma anomalia, aciona o atuador da câmera (o_liga_camera) e sinaliza a tarefa de inspeção.
Os dados processados são enviados para um buffer compartilhado, permitindo que o coletor acesse as informações de forma thread-safe.
Mede o próprio tempo de execução (WCET) para a análise de escalonabilidade.
*/

#include <iostream>
#include <chrono>
#include <atomic>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include "BufferCompartilhado.hpp"
#include "Profiler.hpp"

extern std::atomic<int> i_lidar;
extern std::atomic<bool> e_inspecao;
extern BufferCompartilhado<int> buffer_lidar_coletor;
extern std::mutex mtx_camera;
extern std::condition_variable cv_camera;
extern std::atomic<int> limiar_anomalia;   // configurável pela Operação Remota (Etapa 2)
extern std::atomic<bool> o_liga_camera;     // atuador da câmera (Tabela 1)
extern std::atomic<bool> gatilho_camera;    // gatilho ÚNICO do processamento pesado da câmera
extern MedidorWCET wcet_lidar;              // medidor de tempo de execução

void callback_reconstrucao_teto(boost::asio::steady_timer& timer) {
    auto t0 = std::chrono::steady_clock::now();  // início da medição de WCET

    static int historico[5] = {200, 200, 200, 200, 200};
    static int indice = 0;

    // Rastreia se já estávamos dentro de uma anomalia (para o gatilho de borda)
    static bool buraco_anterior = false;

    // Altura NOMINAL do teto (cm): referência FIXA de "teto normal" do túnel, conhecida
    // do projeto (o histórico da média móvel também é inicializado nela).
    // Por que fixa, e não uma linha de base adaptativa: uma baseline que se atualiza
    // sofria de dois problemas que ligavam a câmera em teto normal — (a) DERIVA: a
    // atualização incremental (EMA) escorregava lentamente para baixo, até o ruído do
    // teto normal cruzar o limiar; e (b) TRAVAMENTO: como a baseline só se atualizava
    // fora de anomalias, qualquer falso positivo a congelava, tornando-se permanente.
    // Como o teto nominal do túnel é constante (anomalias são desvios dele, e a rampa
    // NÃO altera a leitura do teto), a referência fixa é a opção robusta e correta.
    const int TETO_NOMINAL = 200;

    int leitura_atual = i_lidar.load();

    // 1. Média Móvel (reduz o ruído de medição do lidar)
    historico[indice] = leitura_atual;
    indice = (indice + 1) % 5;

    int soma = 0;
    for(int i = 0; i < 5; i++) soma += historico[i];
    int media_movel = soma / 5;

    // 2. Detecção de Anomalia por REGIÃO: desvio do teto suavizado em relação ao teto
    //    NOMINAL fixo (não à média móvel). Assim a anomalia é detectada por TODA a sua
    //    extensão (câmera ligada durante toda a travessia) e o teto normal nunca dispara,
    //    inclusive após rampas. (Comparar com a média móvel só detectava as BORDAS.)
    bool buraco_atual = std::abs(media_movel - TETO_NOMINAL) > limiar_anomalia.load();

    // ESTADO SUSTENTADO: enquanto dentro da anomalia, mantém a câmera ligada e a
    // inspeção ativa (velocidade limitada). Acompanha a região a cada ciclo.
    e_inspecao.store(buraco_atual);
    o_liga_camera.store(buraco_atual);

    // GATILHO ÚNICO do processamento pesado da câmera: só na ENTRADA da anomalia
    // (borda de subida), para não enfileirar trabalho pesado a cada 100ms.
    // O gatilho é setado SOB o mutex da câmera para fechar a janela de "lost wakeup"
    // (garante que a câmera não durma logo após o notify sem ver o gatilho).
    if (buraco_atual == true && buraco_anterior == false) {
        {
            std::lock_guard<std::mutex> lk(mtx_camera);
            gatilho_camera.store(true);
        }
        cv_camera.notify_one();       // acorda a câmera UMA vez, no início da anomalia
    }

    buraco_anterior = buraco_atual; // Salva o estado para o próximo ciclo (100ms)

    // 3. Envio de dados
    buffer_lidar_coletor.push(media_movel);

    wcet_lidar.registrar_desde(t0);  // fim da medição (antes de reagendar)

    // Agendamento Assíncrono
    timer.expires_at(timer.expiry() + std::chrono::milliseconds(100));
    timer.async_wait([&timer](const boost::system::error_code& erro) {
        if (!erro) callback_reconstrucao_teto(timer);
    });
}