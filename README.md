# Robotdog Movement Setup Guide

This guide provides instructions to set up the development environment for the Robotdog Movement project.

---

## Prerequisites

Before proceeding, ensure you have the following installed on your system:
- **Python 3.12** ( Some packages don't work on **python 3.13** or later version )
- **Git**
---

## Steps to Set Up

### 1. Clone the Repository

Clone the project from the Git repository to your local machine:
```bash
git clone https://github.com/tjjd4/robotdog-movement.git
cd robotdog-movement
```

### 2. Create and Activate a Virtual Environment

#### On macOS/Linux:
```bash
python3 -m venv venv
source venv/bin/activate
```

#### On Windows:
```cmd
python -m venv venv
venv\Scripts\activate
```

Once activated, your terminal prompt should display the virtual environment name, e.g., `(venv)`.

#### Set Up `PYTHONPATH`

##### On macOS/Linux:
Modify the `venv/bin/activate` file to include the project root in `PYTHONPATH`. Add the following lines after `export PATH`:
```bash
PYTHONPATH="$VIRTUAL_ENV:$PYTHONPATH"
export PYTHONPATH
```

##### On Windows:
Modify the `venv\Scripts\activate.bat` file. Add the following lines after `set PATH`:
```cmd
set PYTHONPATH=%VIRTUAL_ENV%;%PYTHONPATH%
```

---

### 3. Install Dependencies

Install the required Python packages using the `requirements.txt` file:
```bash
pip install -r requirements.txt
```
---

## Run

To verify everything is working:

1. Activate the virtual environment:
   ```bash
   source venv/bin/activate   # macOS/Linux
   venv\Scripts\activate    # Windows
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
