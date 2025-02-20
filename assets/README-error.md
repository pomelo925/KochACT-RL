# <div align="center"> Common Errors and Solutions </div>



##  <div align="center"> Docker </div>

### 1. Error response from daemon: unknown or invalid runtime name: nvidia 

If you meet this while laumching container, try:
1. Install [NVIDIA Driver](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/), [CUDA Toolkit](https://developer.nvidia.com/downloads), and [NVIDIA Container Tool Kit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

2. configure `nvidia-ctk`.
   
    ```=
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    sudo nvidia-ctk runtime configure --runtime=containerd
    sudo systemctl restart containerd
    ```
