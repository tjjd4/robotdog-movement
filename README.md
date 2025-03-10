# Robotdog Movement Setup Guide

This guide provides instructions to set up the development environment for the Robotdog Movement project using Raspberrypi 5.

---

## Prerequisites

Before proceeding, ensure you have the following installed on your Raspberrypi 5:
- **Python (suggest version: 3.12.7)** ( Some packages don't work on **python 3.13** or later version )
- **Git**

**Note:** Note: If not using Raspberry Pi 5, some hardware-related functions may not work properly, such as the Servo, Camera Module, and Gyroscope Module. In this case, only the `.ipynb` files in the `calculation/` folder can be used to demonstrate the computation process, while other hardware-related functionalities may not function as expected.

( A Linux OS is recommended )

---

## Steps to Set Up
### 1. System Setup

#### Update System Software
```bash
sudo apt update && sudo apt upgrade -y
```

#### Install System packages
```bash
sudo apt install libcamera-dev python3-picamera2
```

#### Enable I2C Connection
```bash
sudo raspi-config
```
Find i2c-interface and choose 'Enable', then restart to apply setting
```bash
sudo reboot
```

### 2. Clone the Repository and Create a Virtual Environment
Clone the project from the Git repository to your local machine:
```bash
git clone https://github.com/tjjd4/robotdog-movement.git
cd robotdog-movement
```

#### On macOS/Linux:
Need system site packages for camera modules ( picamera2 )
```bash
python -m venv --system-site-packages .venv
```

Once activated, your terminal prompt should display the virtual environment name, e.g., `(.venv)`.

#### Set Up `PYTHONPATH`

Modify the `venv/bin/activate` file to include the project root in `PYTHONPATH`. Add the following lines after `export PATH`:
```bash
...
export PATH

# Add the commands bellow
PYTHONPATH="$VIRTUAL_ENV:$PYTHONPATH"
export PYTHONPATH

...
```
re-activate virtual environment!
---

### 3. Install Dependencies

Install the required Python packages using the `requirements.txt` file:
```bash
source .venv/bin/activate
pip install -r requirements.txt
```
---

## Run

To verify everything is working:

1. Activate the virtual environment:
   ```bash
   source .venv/bin/activate
   ```

2. Run a script (e.g., `test_kinematics.py`) to ensure no errors occur:
   ```bash
   python test/test_kinematics.py
   ```

If everything is configured correctly, the script should execute without issues.

---

## Notes

- Ensure you activate the virtual environment in every new terminal session before running the scripts.
- If additional dependencies are added to `requirements.txt`, re-run `pip install -r requirements.txt` to install them.
- If `PYTHONPATH` cannot be set or you prefer not to set it, you can replace any failing absolute imports with relative imports in the code to resolve the errors.
