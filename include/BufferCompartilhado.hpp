#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>

// Usamos template para que o buffer sirva tanto para inteiros (Lidar) 
// quanto para structs mais complexas no futuro
template <typename T>
class BufferCompartilhado {
private:
    std::queue<T> fila;
    std::mutex mtx;
    std::condition_variable cv;
    size_t tamanho_maximo;

public:
    // Construtor: define um limite para a fila não estourar a RAM
    BufferCompartilhado(size_t max = 100) : tamanho_maximo(max) {}

    // Produtor: insere um novo dado na fila
    void push(T item) {
        // Bloqueia o acesso exclusivo a esta região crítica
        std::unique_lock<std::mutex> lock(mtx);
        
        // Política de tempo real: se encher, descartamos a leitura mais antiga.
        // Em robótica, o dado novo do sensor é sempre mais importante que o velho.
        if (fila.size() >= tamanho_maximo) {
            fila.pop(); 
        }
        
        fila.push(item);
        
        // Libera o cadeado ANTES de notificar, para ser mais eficiente
        lock.unlock(); 
        
        // Acorda uma thread (consumidor) que esteja dormindo esperando dados
        cv.notify_one(); 
    }

    // Consumidor: retira o dado mais antigo da fila
    T pop() {
        std::unique_lock<std::mutex> lock(mtx);
        
        // O pulo do gato: se a fila estiver vazia, a thread dorme AQUI.
        // Ela não gasta CPU num loop infinito ("busy wait").
        // Ela só acorda quando o cv.notify_one() for chamado no push().
        cv.wait(lock, [this]() { return !fila.empty(); });
        
        T item = fila.front();
        fila.pop();
        
        return item;
    }
    
    // Função utilitária para debug e logs
    size_t size() {
        std::lock_guard<std::mutex> lock(mtx);
        return fila.size();
    }
};