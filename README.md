# Code for MAXWELL Hangwire and Rotation Table Tests

Code contained in this repo:
- Stepper motor control arduino code for rotation table
- OpenCV code for angular motion detection during hangwire testing

### **Running the OpenCV code:**

(Note that you will need python installed correctly prior to these instructions)

Once you have this repository cloned,
1. Go to the directory where this repo is cloned and run `python3 -m venv venv`
2. Activate the virtual environment
    - Windows: `. venv/Scripts/activate`
    - Mac/Linux: `. venv/bin/activate`
    - You should see `(venv)` appear next to your current directory in the terminal you are using if it is activated

3. Run `pip install -r requirements.txt`
4. Run the python script: `venv/Scripts/python.exe angle_detect.py` (replace with `venv/bin/python.exe` for Mac or Linux)
    - This will prompt the camera to activate, and will show you what it sees. To quit, press `q`.


To deactivate the virtual environment, type `deactivate`.