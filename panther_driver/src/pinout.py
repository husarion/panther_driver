#!/usr/bin/python3

# Define pin names
# MUST BE COMPATIBLE WITH PIN SETTINGS BELOW:
# https://github.com/husarion/panther-config-spec/tree/main/config

# PIN               # Description
VMOT_ON = 6         # Enable mamin power supply to motors (1 - on)
CHRG_SENSE = 7      # Charger sensor (1 - charger plugged in)            
WATCHDOG = 14       # Watchdog pin, if PWM is on tish pin Panther will work
FAN_SW = 15         # Turn on the fan (1 - on)
# SHDN_INIT = 16 Shutdown Init managed by systemd service
AUX_PW_EN = 18      # Enable auxiliary power, eg. supply to robotic arms etc. 
CHRG_EN = 19        # Enable charger
VDIG_OFF = 21       # Turn the digital power off eg. NUC, Router etc. (1 - off)
DRIVER_EN = 23      # Enable motor drivers (1 - on)
E_STOP_RESET = 27   # Works as IN/OUT, IN - gives info if E-stop in on (1 - off), OUT - send 1 to reset estop