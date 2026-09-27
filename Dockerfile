# 假設你使用最新支援 Ubuntu 24.04 的 ROS 2 Jazzy
FROM osrf/ros:jazzy-desktop

# 避免 Python 寫入 pyc 檔並即時輸出終端機訊息
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# 安裝 pip (官方 ROS 映像檔預設不一定有 pip)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# 複製 requirements.txt 並安裝套件
COPY requirements.txt .
# 在容器內，我們可以直接使用 --break-system-packages 來繞過 PEP 668 限制
# 因為這整個容器就是為這個專案服務的，不怕弄髒系統
RUN pip3 install --no-cache-dir -r requirements.txt --break-system-packages

# 設定環境變數：只有在 setup.bash 存在時才 source，避免跳出 No such file or directory
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]
