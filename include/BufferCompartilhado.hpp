/*
BufferCompartilhado.hpp - Buffer thread-safe (fila FIFO) entre tarefas produtoras
e o consumidor.
O que faz: Esta classe é um wrapper em torno de uma fila padrão, protegida por
mutex e variável de condição para garantir segurança em ambiente multithread.
No sistema, liga as tarefas PRODUTORAS (reconstrução do teto/LIDAR e odometria) ao
COLETOR DE DADOS — não tem relação com a câmera, que é acionada por outra variável
de condição (cv_camera).
Suporta push e pop, com política de descarte FIFO quando a capacidade máxima é
atingida (descarta o mais antigo). Inclui try_pop para leitura não bloqueante
(evita a vulnerabilidade TOCTOU) e um mecanismo de fechamento seguro (close) para
encerrar os consumidores de forma elegante, sem deadlock no desligamento.
*/

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <atomic>

template <typename T>
class BufferCompartilhado {
private:
    std::queue<T> fila;
    std::mutex mtx;
    std::condition_variable cv;
    size_t capacidade_maxima;

    // Variáveis para desligamento seguro e métricas
    bool fechado = false;
    std::atomic<int> descartes{0}; 

public:
    explicit BufferCompartilhado(size_t max_size = 200) : capacidade_maxima(max_size) {}

    void push(T dado) {
        std::lock_guard<std::mutex> lock(mtx);
        if (fila.size() >= capacidade_maxima) {
            fila.pop(); // Política FIFO: descarta o mais antigo
            descartes++; // Incrementa o contador de perdas
        }
        fila.push(dado);
        cv.notify_one();
    }

    // O C++17 std::optional permite retornar o dado OU retornar "vazio" (nullopt)
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mtx);
        
        // Dorme até ter algo na fila OU o buffer ser fechado
        cv.wait(lock, [this]() { return !fila.empty() || fechado; });
        
        // Se acordou, está vazio e foi fechado, encerra de forma elegante
        if (fila.empty() && fechado) {
            return std::nullopt; 
        }

        T dado = fila.front();
        fila.pop();
        return dado;
    }

    // Tenta puxar um dado sem bloquear a thread.
    // Resolve a vulnerabilidade TOCTOU (Time-Of-Check to Time-Of-Use)
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mtx); // Tranca a fila
        if (fila.empty()) {
            return std::nullopt; // Retorna vazio imediatamente se não tiver nada
        }
        T dado = fila.front();
        fila.pop();
        return dado;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mtx);
        return fila.size();
    }

    // Método profissional para encerrar consumidores
    void close() {
        std::lock_guard<std::mutex> lock(mtx);
        fechado = true;
        cv.notify_all(); // Acorda todo mundo que estava travado no pop()
    }

    int get_descartes() const {
        return descartes.load();
    }
};