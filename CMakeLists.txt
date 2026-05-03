cmake_minimum_required(VERSION 3.10)
project(RoboInspecaoTunel VERSION 1.0)

# Exigir C++17 para recursos modernos de concorrência
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Encontrar a biblioteca de Threads do Linux (pthreads)
find_package(Threads REQUIRED)

# Incluir a pasta de headers (onde está nosso BufferCompartilhado.hpp)
include_directories(include)

# Adicionar todos os arquivos fonte da pasta src (nosso main.cpp)
file(GLOB SOURCES "src/*.cpp")

# Criar o executável
add_executable(robo_embarcado ${SOURCES})

# Linkar as bibliotecas de threads
target_link_libraries(robo_embarcado PRIVATE Threads::Threads)

