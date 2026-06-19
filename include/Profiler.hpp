/*
Profiler.hpp - Medidor de tempo de execução (WCET) das tarefas.
O que faz: Acumula, de forma thread-safe (atômica), o tempo de execução de uma
tarefa a cada vez que ela roda, guardando o PIOR caso (máximo), a média e o
número de amostras. Esses WCET medidos alimentam a análise de escalonabilidade
da Etapa 2 (cálculo de utilização e do menor período viável).

Uso típico dentro de um callback periódico:
    auto t0 = std::chrono::steady_clock::now();
    ... trabalho da tarefa ...
    medidor.registrar_desde(t0);   // antes de reagendar o timer
*/

#pragma once

#include <atomic>
#include <chrono>
#include <string>
#include <utility>

class MedidorWCET {
    std::atomic<long long> max_ns_{0};
    std::atomic<long long> soma_ns_{0};
    std::atomic<long long> contador_{0};
    std::string nome_;

public:
    explicit MedidorWCET(std::string nome) : nome_(std::move(nome)) {}

    // Registra uma duração já medida, em nanossegundos.
    void registrar(long long ns) {
        soma_ns_.fetch_add(ns, std::memory_order_relaxed);
        contador_.fetch_add(1, std::memory_order_relaxed);

        // Atualiza o máximo (WCET) de forma atômica via compare-and-swap.
        long long anterior = max_ns_.load(std::memory_order_relaxed);
        while (ns > anterior &&
               !max_ns_.compare_exchange_weak(anterior, ns, std::memory_order_relaxed)) {
            // 'anterior' é reescrito pelo CAS a cada falha; repete até vencer.
        }
    }

    // Conveniência: registra o tempo decorrido desde t0 até agora.
    void registrar_desde(std::chrono::steady_clock::time_point t0) {
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t0).count();
        registrar(ns);
    }

    // Resultados (em microssegundos para leitura mais fácil).
    double wcet_us()  const { return max_ns_.load() / 1000.0; }
    double media_us() const {
        long long c = contador_.load();
        return c ? static_cast<double>(soma_ns_.load()) / c / 1000.0 : 0.0;
    }
    long long amostras() const { return contador_.load(); }
    const std::string& nome() const { return nome_; }
};