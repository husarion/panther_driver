# Utilities for eeprom usage

Utils pipeline goals:
- flash eeprom memory with custom config
- automatically load params as environment variables

## First setup:

### Pull repo

```
git clone https://github.com/husarion/panther-config-spec
cd panther-config-spec/utilities/
```

### 1. Install eepromutils

```
./install_eepromutils.sh
```

### 2. Setup upstart

Install upstart to automatically export parameters from `panther_config.yaml` to env variables

```
./setup_upstart.sh
```

## Load config to EEPROM

### 1. Customize panther config file

In order to customize panther config edit the `config/panther_config.yaml`

**Important note:** gpio setting will be loaded according to `robot_ver` in yaml file

### 2. Flash EEPROM

This script flashes files from `config/` dir to EEPROM.

```
./flash_eeprom.sh
```

### 3. Reboot and check

Finally reboot rpi

```
sudo reboot
```

Check if exported variables are correct

```
env | grep PANTHER
```
