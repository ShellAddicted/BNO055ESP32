ARG IDF_VER=v4.4
FROM espressif/idf:${IDF_VER}

SHELL ["/bin/bash", "-c"]

WORKDIR /test/

RUN mkdir -p test_app/components/BNO055ESP32 && \
    mkdir -p test_app/main/ && \
    echo -e "cmake_minimum_required(VERSION 3.5)\ninclude(\$ENV{IDF_PATH}/tools/cmake/project.cmake)\nproject(template-app)" > test_app/CMakeLists.txt && \
    echo -e "idf_component_register(SRCS "main.cpp" INCLUDE_DIRS ".")" > test_app/main/CMakeLists.txt

COPY . test_app/components/BNO055ESP32/

RUN cp test_app/components/BNO055ESP32/examples/example.cpp test_app/main/main.cpp
WORKDIR /test/test_app/
RUN source $IDF_PATH/export.sh && \
    $IDF_PATH/tools/idf.py reconfigure && \
    echo -e "CONFIG_CXX_EXCEPTIONS=y\nCONFIG_COMPILER_CXX_EXCEPTIONS=y\nCONFIG_COMPILER_CXX_EXCEPTIONS_EMG_POOL_SIZE=0\nCONFIG_CXX_EXCEPTIONS=y\nCONFIG_CXX_EXCEPTIONS_EMG_POOL_SIZE=0\n" >> ./sdkconfig && \
    idf.py build