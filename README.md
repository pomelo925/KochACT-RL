# <div align="center"> KochACT-RL </div>

<p align="center">
This repository is built on lerobot framework for the Koch arm and its peripheral devices.
</p>

<br/>

##  <div align="center"> 🌱 Environment 🌱 </div>

* **Arm**: Bimanual Koch Robot Arm v1.1. (2 leaders, 2 followers)
* **Camera**: Intel® RealSense™ Depth Camera D415. (3~5 cams)
* **Computer**: x86 arch with Linux OS.
* **GPU**: GPU is required and must be compatible with CUDA 12.4.

<br/>

##  <div align="center"> 🛠️ Usage 🛠️ </div>

1. Clone & initialize this repo.
   
   ```bash
   git clone https://github.com/hrc-pme/lerobot-koch-actpp.git
   cd lerobot-koch-actpp
   git submodule update --init --recursive
   ```

2. Follow [UdevRules.md](/assets/README-udev.md) to setup Koch's udev rules.

3. Start the container.

   ```bash
   ./gpu_run.sh raw
   ```

   <details> 
      <summary> Mode Options </summary>

      - `raw`: Launch container without running any service.
      - `build`: Build ROS2 workspace.
      - `cali`: Calibrate Koch Robot Arm.
      - `teleop`: Teleoperate Koch Robot Arm.
      - `cam`: Launch all camera nodes.
      - `sync`: Teleoperate, launch camera nodes, and synchronize related topics.
      - `train`: Train with custom dataset.
      - `deploy`: Deploy trained model.
   </details>

<br/>

##  <div align="center"> Issues </div>

<p align="center">
  Before posting issues, please check the troubleshooting steps in  
  <a href="/assets/README-error.md">Common Errors & Solutions.md</a>.
</p>
