cmake_minimum_required(VERSION 3.10)
project(RoboInspecao VERSION 1.0)

# Exige o padrão C++17 (requisito do trabalho)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Bibliotecas de threads e Boost.Asio (tarefas periódicas de tempo real)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Bibliotecas Paho MQTT: cliente C++ (mqttpp3) sobre o cliente assíncrono C
# (mqtt3a), usadas pela ponte MQTT (src/mqtt_bridge.cpp).
find_library(PAHO_MQTTPP NAMES paho-mqttpp3 REQUIRED)
find_library(PAHO_MQTTC  NAMES paho-mqtt3a  REQUIRED)

# Diretório de cabeçalhos (BufferCompartilhado.hpp, Profiler.hpp) + includes do Boost
include_directories(include ${Boost_INCLUDE_DIRS})

# Executável principal: robô embarcado (todas as tarefas em um único binário)
add_executable(robo_embarcado
    src/main.cpp
    src/odometria.cpp
    src/controle.cpp
    src/lidar.cpp
    src/coletor.cpp
    src/comando.cpp
    src/camera.cpp
    src/mock.cpp
    src/mqtt_bridge.cpp    # ponte MQTT (thread dedicada de rede)
)

# Executável de teste unitário do buffer thread-safe
add_executable(testes src/test_buffer.cpp)

# Linka o executável principal com Threads, Boost.Asio e Paho MQTT (C++ e C)
target_link_libraries(robo_embarcado PRIVATE
    Threads::Threads
    Boost::system
    ${PAHO_MQTTPP}
    ${PAHO_MQTTC}
)