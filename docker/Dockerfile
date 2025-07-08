FROM ubuntu:22.04

# Prevent interactive prompts during package installs
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic packages (curl, wget, git, bash, etc.)
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        curl \
        wget \
        git \
        bash \
        build-essential \
        ca-certificates && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ARGoS3 dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        cmake \
        libfreeimage-dev \
        libfreeimageplus-dev \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools \
        freeglut3-dev \
        libxi-dev \
        libxmu-dev \
        liblua5.3-dev \
        lua5.3 \
        doxygen \
        graphviz \
        libgraphviz-dev \
        asciidoc && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ilpincy/argos3.git /home/docker/argos3
RUN cd /home/docker/argos3 &&\
        mkdir build &&\
        cd build &&\
        cmake ../src &&\
        make -j12 &&\
        make doc &&\
        make -j12 install
    
RUN git clone https://github.com/buzz-lang/Buzz.git /home/docker/buzz
RUN cd /home/docker/buzz &&\
    mkdir build && cd build &&\
    cmake ../src &&\
    make -j12 &&\
    make -j12 install &&\
    ldconfig 

RUN git clone https://github.com/ilpincy/argos3-kheperaiv.git /home/docker/argos3_kh4
RUN cd /home/docker/argos3_kh4 &&\
           mkdir build &&\
           cd build &&\
           cmake ../src &&\
           make &&\
           make install &&\
           ldconfig

RUN git clone http://git.mistlab.ca/vvaradharajan/SwarmHaul.git /home/docker/SwarmHaul
RUN cd /home/docker/SwarmHaul/src/Hooks_src &&\
    mkdir build &&\
    cd build &&\
    cmake ../ &&\
    make &&\

    cd /home/docker/SwarmHaul/src/Loop_fun_src/ &&\
    mkdir build &&\
    cd build &&\
    cmake ../ &&\
    make 
        
