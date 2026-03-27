# ─────────────────────────────────────────────────────────────────────────────
# Base image: Espressif's official ESP-IDF Docker image
#   • Includes ESP-IDF v5.3, all Xtensa/RISC-V toolchains, and OpenOCD
#   • Swap the tag (e.g. release-v5.4) when you want to upgrade IDF
#   • Image reference: https://hub.docker.com/r/espressif/idf
# ─────────────────────────────────────────────────────────────────────────────
FROM espressif/idf:release-v5.3

# Prevent interactive prompts during apt installs
ARG DEBIAN_FRONTEND=noninteractive

# ── System dependencies ───────────────────────────────────────────────────────
# git        – version control (also used by idf-component-manager)
# curl/wget  – downloading assets, NodeSource installer
# usbutils   – provides lsusb so you can verify device detection inside container
# libusb-1.0 – USB library required by esptool and OpenOCD
# udev       – device management rules (needed for serial port access on Linux hosts)
# sudo       – lets the container user run privileged commands when needed
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    wget \
    usbutils \
    libusb-1.0-0 \
    udev \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ── Node.js v20 LTS ──────────────────────────────────────────────────────────
# Required for Claude Code CLI.  Uses the official NodeSource setup script.
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# ── Claude Code CLI ──────────────────────────────────────────────────────────
# Installs the `claude` command globally so you can run Claude Code directly
# from the integrated terminal.  Authenticate with `claude` on first use.
RUN npm install -g @anthropic-ai/claude-code

# ── ESP-IDF environment variables ────────────────────────────────────────────
# IDF_PATH        – root of the ESP-IDF checkout (set by the base image, repeated
#                   here so it's visible in ENV for the VS Code extension)
# IDF_TOOLS_PATH  – parent directory where idf_tools.py stores toolchains.
#                   In the espressif/idf image this is /opt/esp (NOT /opt/esp/tools).
#                   The toolchains live under /opt/esp/tools but the env var
#                   must point to the PARENT so idf_tools.py can find them.
# IDF_TARGET      – default chip target; the VS Code extension and idf.py use this.
ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp
ENV IDF_TARGET=esp32s3

# ── Stable Python symlink ─────────────────────────────────────────────────────
# The ESP-IDF VS Code extension needs an explicit pythonBinPath.
# The actual binary lives inside a version-stamped venv
# (e.g. /opt/esp/python_env/idf5.3_py3.11_env/bin/python).
# We resolve that path at build time and create a stable symlink so
# devcontainer.json can reference a fixed path.
RUN PYTHON_BIN=$(ls /opt/esp/python_env/idf*_py*_env/bin/python 2>/dev/null | head -1) && \
    if [ -n "$PYTHON_BIN" ]; then \
        ln -sf "$PYTHON_BIN" /usr/local/bin/idf-python && \
        echo "Linked IDF Python: $PYTHON_BIN → /usr/local/bin/idf-python"; \
    else \
        echo "WARNING: could not find IDF Python venv binary"; \
    fi

# ── Shell initialisation ──────────────────────────────────────────────────────
# Source export.sh in BOTH .bashrc (interactive shells) and .profile (login shells)
# so that `idf.py` is on PATH whether VS Code opens an interactive or login shell.
RUN echo '. "$IDF_PATH/export.sh" > /dev/null 2>&1' >> /root/.bashrc && \
    echo '. "$IDF_PATH/export.sh" > /dev/null 2>&1' >> /root/.profile

# ── Working directory ─────────────────────────────────────────────────────────
# Must match workspaceFolder in devcontainer.json
WORKDIR /workspace
