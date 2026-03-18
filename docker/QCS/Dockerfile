FROM ubuntu:22.04

# Install dependencies (including OpenGL)
RUN apt update && apt install -y \
    wget \
    curl \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libgl1-mesa-glx \
    libglu1-mesa \
    libx11-6 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxrender1 \
    libxrandr2 \
    libxi6 \
    libxinerama1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libpulse0 \
    sudo \
    fuse


RUN useradd -ms /bin/bash qgcuser
RUN usermod -a -G dialout qgcuser
USER qgcuser
WORKDIR /home/qgcuser


RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage

RUN chmod +x QGroundControl-x86_64.AppImage

CMD ["./QGroundControl-x86_64.AppImage"]
