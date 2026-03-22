FROM ubuntu:22.04
LABEL maintainer="paparazzi-gazebo-classic-sim"

# ── Init ─────────────────────────────────────────────────────────────
ENV TINI_VERSION v0.19.0
ADD https://github.com/krallin/tini/releases/download/${TINI_VERSION}/tini /tini
RUN chmod +x /tini

# ── Locale & timezone ────────────────────────────────────────────────
ENV LANG C.UTF-8
ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive
RUN ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
    && apt-get update \
    && apt-get install -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# ── Repositories ─────────────────────────────────────────────────────
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        software-properties-common curl lsb-release gnupg git ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    # Paparazzi PPA
    && add-apt-repository -y ppa:paparazzi-uav/ppa \
    # kisak-mesa: Mesa 24.x backport — required for Intel Arc (Meteor Lake) iris driver
    && add-apt-repository -y ppa:kisak/kisak-mesa \
    # OSRF: Gazebo Classic 11
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list

# ── All packages in one layer ─────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Paparazzi build & GCS
    paparazzi-dev \
    pprzgcs \
    # Build tools (simulation only — ARM cross-compiler not needed)
    build-essential \
    cmake \
    python-is-python3 \
    python3-future \
    python3-lxml \
    python3-numpy \
    python3-pip \
    python3-opencv \
    python3-opengl \
    python3-pyqt5 \
    python3-pyqt5.qtwebengine \
    python3-pyqtgraph \
    python3-scipy \
    python3-serial \
    python3-matplotlib \
    # Unified Mocap Router
    libboost-program-options-dev \
    libboost-filesystem-dev \
    # Gazebo Classic 11
    gazebo \
    libgazebo-dev \
    # Mesa 24.x for Intel Arc GPU (iris driver)
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    # X11 / GUI
    x11-apps \
    dbus-x11 \
    libcanberra-gtk3-module \
    # Joystick
    joystick \
    libsdl2-dev \
    # USB & IMU hardware support
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    # Audio (PulseAudio client only — server runs on host)
    libpulse0 \
    pulseaudio-utils \
    # Compiler for BEBOP
    gcc-arm-linux-gnueabi \
    g++-arm-linux-gnueabi \
    libc6-dev-armel-cross \
    linux-libc-dev-armel-cross \
    binutils-arm-linux-gnueabi \
    # Video (RTP viewer)
    vlc \
    # Speech synthesis (PprzGCS uses --speech for command confirmations)
    speech-dispatcher \
    espeak-ng \
    # Permission helper
    gosu \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y libjpeg-turbo8-dev libpng-dev libtiff-dev zlib1g-dev libdc1394-dev && rm -rf /var/lib/apt/lists/*

# Install Python dependencies matching the repo virtualenv requirements
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir 'empy==3.3.4' \
    && awk '!/^(PyQt5|PyQtWebEngine|pyqtgraph|scipy|pyserial|PyOpenGL|matplotlib)$/' /tmp/requirements.txt > /tmp/requirements.pip.txt \
    && python3 -m pip install --no-cache-dir -r /tmp/requirements.pip.txt \
    && printf '%s\n' \
       'import importlib' \
       'modules = ["lxml", "numpy", "scipy", "ivy", "serial", "PyQt5.QtWidgets", "PyQt5.QtWebEngineWidgets", "pyqtgraph", "OpenGL", "unidecode", "matplotlib", "distro", "em", "pexpect", "setuptools"]' \
       'missing = []' \
       'for module in modules:' \
       '    try:' \
       '        importlib.import_module(module)' \
       '    except Exception as exc:' \
       '        missing.append(f"{module}: {exc}")' \
       'if missing:' \
       '    raise SystemExit("Missing Python modules:\\n" + "\\n".join(missing))' \
       'print("All required Python modules are importable.")' \
       > /tmp/verify_python_deps.py \
    && python3 /tmp/verify_python_deps.py \
    && rm -f /tmp/verify_python_deps.py \
    && rm -f /tmp/requirements.pip.txt \
    && rm -f /tmp/requirements.txt

# ── Runtime environment ───────────────────────────────────────────────
ENV PULSE_SERVER=/run/pulse/native

# ── User setup ───────────────────────────────────────────────────────
ENV USERNAME=pprz
ENV USER_ID=1000
RUN groupadd -f input \
    && groupadd -f usb \
    && useradd --shell /bin/bash -u $USER_ID -o -c "Paparazzi Docker user" -m $USERNAME \
    && usermod -aG sudo,dialout,plugdev,input,usb $USERNAME

COPY docker/gazebo-classic/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/tini", "--", "/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
