1. Setup    (1) The main.cpp is at src/model_depoly and python code is at 
                src/model_depoly/wifi_mqtt
            (2) To start running the code, please enter
                $ sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM -f
                under src/model_depoly to start compiling
            (3) After compiling the program, wait until LED2 on the mbed
                Microcontroller ligth up. Then enter
                $ sudo python3 wifi_mqtt/mqtt_client.py
                under src/model_depoly to start python code.
2. Run      (1) After you start the python code, the mbed will enter GestureUI
                mode.  Then, you can do different gesture
            (2) python will recieve the result of gesture. Then, after doing five gesture
                python will write the command to stop Gesture detect, and get the result 
                of feature.
