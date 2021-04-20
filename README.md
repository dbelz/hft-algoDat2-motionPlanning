# Data Structures and Algorithms 2 - Motion Planning Pre-Exam
Pre-exam for the lecture _Data Structures & Algorithms 2_ at the HFT Stuttgart (summer semester 2021) by Dr. Daniel Schneider from Mercedes-Benz AG.

## MotionPlanning Template

This a python template for testing out some MP algorithms. Tested with Python 3.7.8. But should run on any newer version (update requirements.txt). Only packages needed are numpy and image.

Tk is also needed for the GUI, but there is no suitable pip package. On Linux, Tk can be installed using:

    $ sudo apt install python3-tk

### Installation
-----------------

You should create a virtual environment and install the required packages with the following commands:

**Windows:**

    python -m venv env
    .\env\Scripts\activate    
    (env) $ pip install -r requirements.txt


**Linux:**

    python3 -m venv env
    source env/bin/activate
    (env) $ pip install -r requirements.txt

### Run
------------

In order to run it make sure that your venv is runnig and then

**Windows:**

    $ .\env\Scripts\activate 
    (env) $ python app.py

**Linux:**

    source env/bin/activate
    (env) $ python app.py

