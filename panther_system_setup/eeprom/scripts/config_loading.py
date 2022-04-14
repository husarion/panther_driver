#!/usr/bin/python3

import os
import yaml
from builtins import str
from typing import List

def parse_yaml_to_list(d, keys=()):
    if type(d) == dict:
         for k in d:
            for rv in parse_yaml_to_list(d[k], keys + (k, )):
                yield rv
    else:
        yield (keys, d)

def key_string_to_env_name(keys: List[str]):
    env_name = ""
    for i in range(len(keys)-1):
        env_name += keys[i].upper() + "_"

    env_name += keys[-1].upper()

    return env_name

def main():
    config_dir = "/proc/device-tree/hat/custom_0"
    env_profile_file_dir = "/etc/profile.d/panther_env_export.sh"
    env_systemd_file_dir = "/etc/systemd/system/panther.env"

    if not os.path.isfile(config_dir):
        print("No config file, loadind default")

        with open(env_profile_file_dir, "w") as file:
            file.write("#!/bin/sh\n")
            file.write("export PANTHER_HAS_CONFIG=false\n")

        with open(env_systemd_file_dir, "w") as file:
            file.write("PANTHER_HAS_CONFIG=false\n")

        return 0

    with open(config_dir, "r") as stream:
        try:
            config_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return -1

    
    env_list = []

    for key_list, val in parse_yaml_to_list(config_file):
        env_name = key_string_to_env_name(key_list)
        env_list.append((env_name, val))
        # print('{}={}'.format(env_name, val))

    with open(env_profile_file_dir, "w") as file:
        file.write("#!/bin/sh\n")

        for e in env_list:
            file.write("export {}={}\n".format(e[0], e[1]))

        file.write("export PANTHER_HAS_CONFIG=true\n")

    with open(env_systemd_file_dir, "w") as file:
        for e in env_list:
            file.write("{}={}\n".format(e[0], e[1]))

        file.write("PANTHER_HAS_CONFIG=true\n")

if __name__ == "__main__":
    main()
