cmake_minimum_required(VERSION 3.10)
project(RoboInspecao VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# --- NOVO: localiza as bibliotecas Paho (C++ e C) ---
find_library(PAHO_MQTTPP NAMES paho-mqttpp3 REQUIRED)
find_library(PAHO_MQTTC  NAMES paho-mqtt3a  REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS})

add_executable(robo_embarcado
    src/main.cpp
    src/odometria.cpp
    src/controle.cpp
    src/lidar.cpp
    src/coletor.cpp
    src/comando.cpp
    src/camera.cpp
    src/mock.cpp
    src/mqtt_bridge.cpp    # <-- NOVO: a thread de rede (vamos criar no próximo passo)
)

add_executable(testes src/test_buffer.cpp)

# --- ALTERADO: linka também o Paho no executável principal ---
target_link_libraries(robo_embarcado PRIVATE
    Threads::Threads
    Boost::system
    ${PAHO_MQTTPP}
    ${PAHO_MQTTC}
)