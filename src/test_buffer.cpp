/* 
Teste Unitário do Buffer Compartilhado 
Este teste é projetado para validar as funcionalidades principais do BufferCompartilhado, incluindo:
1. Operações básicas de push e pop.
2. Comportamento de descarte quando a capacidade máxima é atingida.
3. Fechamento seguro do buffer para garantir que consumidores não fiquem bloqueados indefinidamente.
*/

#include <iostream>
#include <cassert>
#include "BufferCompartilhado.hpp"

// Função simples para rodar testes isolados no buffer
int main() {
    std::cout << "--- INICIANDO TESTES UNITARIOS DO BUFFER ---\n";
    
    BufferCompartilhado<int> buffer_teste(3); // Fila minúscula de 3 posições

    // Teste 1: push e pop normais
    buffer_teste.push(10);
    buffer_teste.push(20);
    assert(buffer_teste.size() == 2);
    assert(buffer_teste.try_pop().value() == 10);

    // Teste 2: Descarte (Overflow)
    buffer_teste.push(30);
    buffer_teste.push(40);
    buffer_teste.push(50); // Aqui a fila enche (20, 30, 40) e joga o 20 fora para caber o 50
    
    assert(buffer_teste.get_descartes() == 1); // Garante que o contador de métricas funciona
    
    // Teste 3: Fechamento Seguro (Close)
    buffer_teste.close();
    auto resultado = buffer_teste.pop(); 
    // Como está fechado, o pop não bloqueia, ele retorna as coisas velhas e depois nullopt
    
    std::cout << "--- TODOS OS TESTES PASSARAM COM SUCESSO! ---\n";
    return 0;
}