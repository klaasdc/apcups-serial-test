apcups-serial-test
==================

This is a test application to communicate with an APC SMC1000i UPS over the serial port.
The protocol (Microlink) does not seem to be documented, but references to some parameters and fields can be found in various APC datasheets/manuals.
For more details on the development, visit https://sites.google.com/site/klaasdc/apc-smartups-decode

The code is written for Python 3, and uses pyserial to interface with an USB-to-serial adapter. The only dependecy is the pyserial library.

Running the program
-------------------
Run the python script with the serial port as argument:
```
python3 apcups-serial-test.py COM5

python3 apcups-serial-test.py /dev/ttyUSB0

python3 apcups-serial-test.py /dev/ttyS0
```

When running the program, a simple CLI is started.
Type 'commstate' to see the actual state.
- INIT or INIT_RESET means the communication is not yet established
- MODE0 means that the UPS is looping over its message IDs for the first time
- MODE1 means that the UPS is now sending status updates continuously

Type 'all' to get all the known parameters from the internal state dictionary.

Note: This is all just work in progress 

Troubleshooting
---------------
If you get a Permission Denied error on opening the serial port, you might need to add your user to the dialout group.
```
sudo usermod -a -G dialout YOUR_USER_NAME
```
