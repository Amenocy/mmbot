FROM debian:latest

RUN apt-get update && apt-get install -y \
    git \
    cmake \
    make \
    g++ \
    libssl-dev \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /home/mmbotuser

RUN git clone https://github.com/Amenocy/mmbot.git
WORKDIR /home/mmbotuser/mmbot
RUN git checkout feat/mylocalbroker

RUN ./update

ENV TUNNEL_PORT 7667

EXPOSE ${TUNNEL_PORT}

CMD ["sh", "-c", "bin/mmbot -p ${TUNNEL_PORT} start & tail -f /dev/null"]