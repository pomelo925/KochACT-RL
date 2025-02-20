# <div align="center"> lerobot-koch-actpp </div>

This repository is built on the [lerobot](https://github.com/hrc-pme/lerobot/tree/33724a273dfa3a62b845cbbb030b21b71fc5d12b) framework to establish a software environment for the Koch arm and its peripheral devices.

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

3. (Optional) [Setup your huggingface token](https://huggingface.co/docs/hub/security-tokens) in `.env` file.
   
   ```bash
   cd /docker/entrypoint
   cp .env.example .env
   ```

4. Start the container.

   ```bash
   ./gpu_run.sh raw
   ```

   <details> 
      <summary> spoiler Mode Options </summary>
   
      - We provide 8 modes: `raw`, `ws-build`, `arm-cali`, `arm-sync`, `armv-sync`, `armv-rec`, `train`, and `deploy`.   
      - You can check details under `/docker/entrypoint`.
   </details>

<br/>

##  <div align="center"> ⚠️ Issues ⚠️ </div>

Before posting issues, please ensure you have attempted the troubleshooting steps outlined in [Common Errors & Solutions.md](/assets/README-error.md).
