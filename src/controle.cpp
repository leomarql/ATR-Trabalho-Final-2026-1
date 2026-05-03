#include <thread>
#include <chrono>
#include <atomic>

extern std::atomic<double> j_sp_velocidade;
extern std::atomic<double> velocidade_atual;
extern std::atomic<int> o_aceleracao;

void tarefa_controle_navegacao() {
    const auto periodo = std::chrono::milliseconds(80);
    auto proximo_ciclo = std::chrono::steady_clock::now() + periodo;
    double Kp = 2.0, Ki = 0.5, Kd = 0.1;
    double erro_integral = 0.0;
    double erro_anterior = 0.0;
    double dt = 0.080; 

    while(true) {
        double sp = j_sp_velocidade.load();
        double pv = velocidade_atual.load();
        double erro = sp - pv;
        erro_integral += erro * dt;
        double derivada = (erro - erro_anterior) / dt;
        
        double saida = (Kp * erro) + (Ki * erro_integral) + (Kd * derivada);
        
        int saida_saturada = static_cast<int>(saida);
        if (saida_saturada > 100) saida_saturada = 100;
        if (saida_saturada < -100) saida_saturada = -100;

        o_aceleracao.store(saida_saturada);
        erro_anterior = erro;

        std::this_thread::sleep_until(proximo_ciclo);
        proximo_ciclo += periodo;
    }
}