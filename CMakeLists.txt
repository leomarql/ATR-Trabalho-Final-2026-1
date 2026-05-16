cmake_minimum_required(VERSION 3.10)
project(RoboInspecao VERSION 1.0)

# Exige o padrão C++17 (Requisito do trabalho)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Procura as bibliotecas de Threads e do Boost no sistema (WSL)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Define onde estão os arquivos de cabeçalho (ex: BufferCompartilhado.hpp)
include_directories(include ${Boost_INCLUDE_DIRS})

# ---> A LINHA QUE VOCÊ DEVE ALTERAR <---
# Adiciona TODOS os arquivos .cpp para serem compilados juntos em um único executável
add_executable(robo_embarcado 
    src/main.cpp 
    src/odometria.cpp 
    src/controle.cpp 
    src/lidar.cpp 
    src/coletor.cpp
    src/comando.cpp
    src/camera.cpp
    src/mock.cpp
    
)

add_executable(testes src/test_buffer.cpp)

# Linka o executável com as bibliotecas do Linux e do Boost
target_link_libraries(robo_embarcado PRIVATE Threads::Threads Boost::system)