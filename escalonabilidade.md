# Análise de Tempo de Resposta e Escalonabilidade

## 1. Objetivo

Este capítulo responde à questão central exigida pelo enunciado: *para um dado
sistema computacional, qual o menor tempo (período) que podemos utilizar nas
tarefas cíclicas de modo a garantir a escalonabilidade do sistema?*

A análise se baseia em **WCET (Worst-Case Execution Time) medidos**, obtidos por
instrumentação direta do código embarcado, e não em estimativas teóricas. Cada
tarefa periódica cronometra o próprio tempo de execução a cada ciclo (classe
`MedidorWCET`), acumulando o máximo, a média e o número de amostras, impressos ao
final da execução.

## 2. Metodologia de medição

Cada callback periódico marca o instante inicial no topo e registra o tempo
decorrido imediatamente antes de reagendar o timer, de modo que a medição
abrange apenas o trabalho computacional da tarefa, sem incluir o custo do
reagendamento. A medição é acumulada de forma atômica (`std::atomic` com
`memory_order_relaxed` e atualização do máximo por *compare-and-swap*), o que
mantém o overhead da instrumentação desprezível e seguro em ambiente
multithread.

Os dados foram coletados em três execuções independentes do sistema completo em
modo online (robô + simulador físico via MQTT), sob carga real, incluindo os
disparos da câmera nas anomalias do túnel.

## 3. WCET medidos

A tabela consolida as três execuções. Adota-se como WCET de projeto o **maior
valor observado** entre as rodadas (critério conservador).

| Tarefa     | Período T (µs) | WCET rod. 1 (µs) | WCET rod. 2 (µs) | WCET rod. 3 (µs) | WCET adotado (µs) | Média típica (µs) |
|------------|----------------|------------------|------------------|------------------|-------------------|-------------------|
| Odometria  | 20 000         | 578,2            | 361,0            | 178,2            | **578,2**         | ~2,5              |
| Controle   | 80 000         | 24,6             | 53,8             | 25,6             | **53,8**          | ~0,5              |
| Lidar      | 100 000        | 371,4            | 169,4            | 146,9            | **371,4**         | ~22,6             |
| Comando    | 200 000        | 24,3             | 13,3             | 41,0             | **41,0**          | ~0,4              |
| Câmera*    | (evento)       | 1 268 405        | 1 018 395        | 1 171 890        | **~1 268 405**    | ~1 100 000        |

\* A câmera é uma tarefa **orientada a evento** (não periódica), executada em
thread nativa dedicada. Não integra o conjunto periódico analisado por Rate
Monotonic; está listada apenas para evidenciar sua magnitude.

**Observação sobre WCET vs. média.** Há uma diferença grande entre o WCET e a
média de cada tarefa periódica (ex.: odometria com média ~2,5 µs e pico de
578 µs). Essa diferença não vem da tarefa, e sim do sistema operacional de
propósito geral (Linux/WSL): preempção, troca de contexto e *cache misses*
ocasionais inflam o pior caso medido. Justamente por isso a análise de
escalonabilidade adota o pior caso, mais conservador, e não a média.

## 4. Utilização do processador

A utilização de uma tarefa é Uᵢ = Cᵢ / Tᵢ, e a utilização total do conjunto
periódico é U = Σ Uᵢ. Usando os WCET adotados:

| Tarefa    | Cᵢ (µs) | Tᵢ (µs) | Uᵢ = Cᵢ/Tᵢ |
|-----------|---------|---------|------------|
| Odometria | 578,2   | 20 000  | 0,02891    |
| Controle  | 53,8    | 80 000  | 0,00067    |
| Lidar     | 371,4   | 100 000 | 0,00371    |
| Comando   | 41,0    | 200 000 | 0,00021    |
| **Total** |         |         | **≈ 0,0335** |

A utilização total do conjunto periódico é de aproximadamente **3,35% da
capacidade de um núcleo**. O sistema está muito longe da saturação.

## 5. Teste de escalonabilidade — Rate Monotonic (RM)

Sob escalonamento Rate Monotonic (prioridade inversamente proporcional ao
período), o limite de utilização suficiente de Liu & Layland para n tarefas é:

> U ≤ n · (2^(1/n) − 1)

Para n = 4 tarefas periódicas:

> U_limite = 4 · (2^(1/4) − 1) ≈ 4 · (1,1892 − 1) ≈ **0,757**

Como U ≈ 0,0335 ≪ 0,757, o conjunto é **escalonável por Rate Monotonic com folga
de mais de 20×**. Por ser um teste *suficiente* (não necessário), aprovar com tal
margem encerra a questão da viabilidade — não é necessário recorrer à Análise de
Tempo de Resposta exata (RTA) para confirmar.

## 6. Resposta: menor período viável

A pergunta do enunciado é qual o menor período que ainda garante
escalonabilidade. Há dois limites a considerar.

**Limite por utilização agregada.** Escalando todos os períodos por um mesmo
fator k (preservando as proporções), a utilização vira U(k) = 0,0335 / k.
Igualando ao limite RM:

> 0,0335 / k = 0,757  ⟹  k ≈ 0,044

Ou seja, todos os períodos poderiam ser reduzidos a ~4,4% dos atuais e o conjunto
ainda passaria no teste RM. Isso levaria a odometria de 20 ms para ~0,9 ms.

**Limite por execução individual (o gargalo real).** A condição mínima é
Cᵢ < Tᵢ: nenhuma tarefa pode ter período menor que o próprio tempo de execução.
O caso mais restritivo é a odometria, cujo WCET adotado é 578 µs; seu período não
pode ser inferior a ~600 µs sob pena de o tempo de execução não caber em um ciclo.

**Conclusão.** O menor período viável **não é limitado pela utilização agregada**
(que oferece folga enorme), e sim pelo **WCET da tarefa individual mais carregada
em relação ao seu período**. Para a configuração atual, a odometria impõe o piso
prático em torno de ~0,6–0,9 ms.

## 7. Ressalva sobre o modelo de execução (monocore vs. multicore)

Toda a análise das seções 4–6 adota o modelo clássico de **um único núcleo** com
preempção, em que a utilização escalonável é limitada a 1 (ou ~0,757 por RM).

O sistema implementado, porém, **não** executa nesse modelo: as tarefas
periódicas são despachadas por um *thread pool* de **4 threads** do Boost.Asio
(`io.run()` em quatro threads), o que caracteriza execução **paralela** e não
preempção em um único núcleo. Em um modelo de m núcleos, a utilização total
escalonável tende a m (e não a 1).

Esta ressalva é intelectualmente honesta e relevante: o limite Rate Monotonic
monocore é o **caso mais conservador** (mais restritivo). Como o sistema passa
nele com folga superior a 20×, passa com folga ainda maior no modelo multicore
real em que de fato roda. A conclusão de escalonabilidade, portanto, é robusta
nos dois modelos.

## 8. Tarefas não periódicas (câmera e coletor)

A câmera (WCET ≈ 1,27 s) e o coletor de dados são tarefas **orientadas a evento**,
executadas em threads nativas dedicadas, e por isso **não integram** o conjunto
periódico Rate Monotonic.

A magnitude da câmera evidencia a razão de projeto para essa separação: com
~1,27 segundo de processamento (multiplicação de matrizes O(N³)), se a câmera
fosse uma tarefa periódica no mesmo núcleo das tarefas de tempo real, estouraria
qualquer deadline e inviabilizaria a previsibilidade das tarefas rápidas.
Isolá-la em uma thread nativa, dentro de um pool de 4 núcleos, permite ao sistema
absorver essa carga pesada sem comprometer os deadlines das tarefas cíclicas — é
exatamente o que a separação produtor/consumidor e o uso de threads dedicadas
para I/O e processamento pesado garantem.

## 9. Síntese

O conjunto periódico utiliza ~3,35% de um núcleo e passa no teste de Rate
Monotonic (limite ~0,757) com folga superior a 20×. O menor período viável é
limitado pelo WCET da tarefa individual (a odometria, ~0,6 ms), não pela
utilização agregada. A análise monocore é o caso conservador; o sistema, que roda
em um pool multicore, é escalonável com margem ainda maior. A separação da câmera
(~1,27 s) e do coletor em threads nativas dedicadas é o que preserva a
previsibilidade das tarefas de tempo real.