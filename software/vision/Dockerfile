# Vision: YOLO from ONNX on the CPU. Two targets, both from the repo root after building vector:
#   docker build -f software/vision/Dockerfile --target export -t vector-yolo-export .
#   docker run --rm -v docker_models:/models vector-yolo-export          # yolo26n(-seg) -> .onnx, once
#   docker build -f software/vision/Dockerfile -t vector-vision .
# PyTorch and Ultralytics are only in the export image; the robot runs ONNX Runtime + NumPy.

FROM vector AS export
RUN apt-get update && apt-get install -y --no-install-recommends python3-venv && rm -rf /var/lib/apt/lists/*
# CPU-only PyTorch: PyPI's ARM64 wheels drag in 5.8 GB of CUDA
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    && /opt/venv/bin/pip install --no-cache-dir ultralytics onnx onnxslim \
    && /opt/venv/bin/pip uninstall -y opencv-python \
    && /opt/venv/bin/pip install --no-cache-dir opencv-python-headless
ENV PATH=/opt/venv/bin:$PATH
WORKDIR /models
CMD ["bash", "-c", "for m in yolo26n yolo26n-seg; do for s in ${SIZES:-320 480}; do yolo export model=$m.pt format=onnx imgsz=$s simplify=True >/dev/null && mv $m.onnx $m-$s.onnx; done; done; ls -l *.onnx"]

FROM vector AS vision
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-vision-msgs python3-venv ffmpeg \
    && rm -rf /var/lib/apt/lists/*
# a venv that still sees rclpy; NumPy stays 1.x like the ROS message packages expect
RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir "numpy<2" onnxruntime "opencv-python-headless<4.11"
ENV PATH=/opt/venv/bin:$PATH
COPY software/vision/yolo_node.py software/vision/yolo_onnx.py /ws/vision/
WORKDIR /ws/vision
