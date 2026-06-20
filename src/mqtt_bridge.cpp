/*
mqtt_bridge.cpp - Ponte MQTT (thread dedicada de rede).
O que faz: Roda em uma thread nativa SEPARADA do thread pool do Boost.Asio, para
que a latência/jitter da rede NUNCA contamine as tarefas de tempo real (odometria
20ms, controle 80ms, lidar 100ms).

Responsabilidades (conforme contrato_api.md):
  - ASSINA  robo/sensores  -> escreve i_encoder, i_sentido, i_lidar
  - ASSINA  robo/comandos  -> escreve e_automatico, j_sp_velocidade, limiar_anomalia, freio_ativo
  - PUBLICA robo/atuadores -> le o_aceleracao
  - PUBLICA robo/telemetria-> le posicao_x, i_lidar, confianca, velocidade, modo, etc.

A comunicação com o resto do sistema é feita exclusivamente pelas variáveis
atômicas globais, sem mutex e sem tocar nos callbacks do Asio.
*/

#include <iostream>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// --- Variáveis Globais (nascem no main.cpp) ---
extern std::atomic<bool>   executando;
extern std::atomic<bool>   e_automatico;
extern std::atomic<bool>   i_encoder;
extern std::atomic<int>    i_sentido;     // sentido do movimento (+1/-1/0)
extern std::atomic<double> j_sp_velocidade;
extern std::atomic<double> velocidade_atual;
extern std::atomic<int>    o_aceleracao;
extern std::atomic<int>    i_lidar;
extern std::atomic<bool>   e_inspecao;
extern std::atomic<int>    limiar_anomalia;
extern std::atomic<bool>   o_liga_camera;
extern std::atomic<int>    confianca_atual;
extern std::atomic<double> posicao_x;     // exposto pela odometria para a telemetria
extern std::atomic<bool>   freio_ativo;   // parada firme (comando "parar")

// --- Configuração (deve bater com contrato_api.md) ---
const std::string SERVIDOR         {"tcp://localhost:1883"};
const std::string CLIENT_ID        {"robo_embarcado"};
const std::string TOPICO_SENSORES  {"robo/sensores"};
const std::string TOPICO_COMANDOS  {"robo/comandos"};
const std::string TOPICO_ATUADORES {"robo/atuadores"};
const std::string TOPICO_TELEMETRIA{"robo/telemetria"};

// --- Callback: trata conexão e chegada de mensagens ---
class CallbackMqtt : public virtual mqtt::callback {
    mqtt::async_client& cli_;
    double modulo_manual_ = 5.0; // módulo do setpoint manual (último set_velocidade)

public:
    explicit CallbackMqtt(mqtt::async_client& cli) : cli_(cli) {}

    // Chamado ao conectar (e a cada reconexão automática): (re)assina os tópicos
    void connected(const std::string& /*cause*/) override {
        std::cout << "[MQTT] Conectado. Assinando " << TOPICO_SENSORES
                  << " e " << TOPICO_COMANDOS << "\n";
        cli_.subscribe(TOPICO_SENSORES, 0);
        cli_.subscribe(TOPICO_COMANDOS, 0);
    }

    void connection_lost(const std::string& cause) override {
        std::cerr << "[MQTT] Conexao perdida: " << cause << " (tentando reconectar)\n";
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        const std::string& topico  = msg->get_topic();
        const std::string  payload = msg->to_string();

        try {
            auto j = json::parse(payload);

            // --- SENSORES (simulador -> robô) ---
            if (topico == TOPICO_SENSORES) {
                if (j.contains("encoder")) i_encoder.store(j["encoder"].get<bool>());
                if (j.contains("sentido")) i_sentido.store(j["sentido"].get<int>());
                if (j.contains("lidar"))   i_lidar.store(j["lidar"].get<int>());
            }
            // --- COMANDOS (operação remota -> robô) ---
            else if (topico == TOPICO_COMANDOS) {
                std::string cmd = j.value("comando", std::string(""));

                if (cmd == "set_modo") {
                    std::string v = j.value("valor", std::string("auto"));
                    bool em_auto = (v == "auto");
                    e_automatico.store(em_auto);
                    // Auto: solta o freio (lógica autônoma assume).
                    // Manual: começa PARADO por segurança, até o operador comandar direção.
                    freio_ativo.store(!em_auto);
                }
                else if (cmd == "set_velocidade") {
                    double val = j.value("valor", 5.0);
                    modulo_manual_ = val;          // guarda módulo para direita/esquerda
                    j_sp_velocidade.store(val);
                    // Não mexe no freio: só configura a velocidade. O robô só anda
                    // quando o operador comandar uma direção.
                }
                else if (cmd == "set_limiar") {
                    limiar_anomalia.store(j.value("valor", 10));
                }
                else if (cmd == "direcao") {
                    std::string v = j.value("valor", std::string("parar"));
                    if (v == "direita") {
                        j_sp_velocidade.store(modulo_manual_);
                        freio_ativo.store(false);   // solta o freio e anda (avanço)
                    }
                    else if (v == "esquerda") {
                        j_sp_velocidade.store(-modulo_manual_);
                        freio_ativo.store(false);   // solta o freio e anda (recuo)
                    }
                    else if (v == "parar") {
                        j_sp_velocidade.store(0.0);
                        freio_ativo.store(true);    // FREIO: parada firme
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[MQTT] Mensagem ignorada (JSON invalido) em "
                      << topico << ": " << e.what() << "\n";
        }
    }
};

void tarefa_mqtt_bridge() {
    mqtt::async_client client(SERVIDOR, CLIENT_ID);
    CallbackMqtt cb(client);
    client.set_callback(cb);

    auto connOpts = mqtt::connect_options_builder()
        .clean_session()
        .automatic_reconnect(std::chrono::seconds(1), std::chrono::seconds(10))
        .finalize();

    // --- Conexão inicial ---
    try {
        std::cout << "[MQTT] Conectando ao broker " << SERVIDOR << " ...\n";
        client.connect(connOpts)->wait();
    } catch (const mqtt::exception& e) {
        std::cerr << "[MQTT] Falha ao conectar: " << e.what()
                  << "\n[MQTT] A bridge nao subira. Verifique se o mosquitto esta rodando.\n";
        return;
    }

    // --- Loop de publicação (telemetria + atuadores) ---
    while (executando.load()) {
        try {
            json tele;
            tele["x"]           = posicao_x.load();
            tele["y"]           = i_lidar.load();
            tele["confianca"]   = confianca_atual.load();
            tele["velocidade"]  = velocidade_atual.load();
            tele["modo"]        = e_automatico.load() ? "auto" : "manual";
            tele["inspecao"]    = e_inspecao.load();
            tele["liga_camera"] = o_liga_camera.load();
            client.publish(TOPICO_TELEMETRIA, tele.dump(), 0, false);

            json atu;
            atu["aceleracao"] = o_aceleracao.load();
            client.publish(TOPICO_ATUADORES, atu.dump(), 0, false);
        } catch (const mqtt::exception& e) {
            std::cerr << "[MQTT] Erro ao publicar: " << e.what() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // --- Encerramento limpo ---
    try {
        std::cout << "[MQTT] Desconectando...\n";
        client.disconnect()->wait();
    } catch (const mqtt::exception& e) {
        std::cerr << "[MQTT] Erro ao desconectar: " << e.what() << "\n";
    }
}