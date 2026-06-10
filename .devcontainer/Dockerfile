FROM mcr.microsoft.com/devcontainers/base:trixie

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
        git \
        ca-certificates \
        unzip \
        build-essential \
        python3 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz \
    | tar -xz -C /tmp \
    && find /tmp -maxdepth 2 -type f -name arduino-cli -exec install -m 0755 {} /usr/local/bin/arduino-cli \; \
    && rm -rf /tmp/arduino-cli* /tmp/*arduino-cli*

ENV PATH="/usr/local/bin:${PATH}"
