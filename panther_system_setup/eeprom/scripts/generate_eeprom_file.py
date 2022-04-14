
import yaml
import sys
import subprocess
from typing import List


def parse_yaml_to_list(d, keys=()):
    if type(d) == dict:
         for k in d:
            for rv in parse_yaml_to_list(d[k], keys + (k, )):
                yield rv
    else:
        yield (keys, d)


def get_hat_txt_config(config_file):
    panther_version = config_file["panther"]["robot_version"]

    known_versions = ["1.05", "1.20"]

    if panther_version not in known_versions:
        print("Unknown panther version {}, known versions are: {}", panther_version ,known_versions)
        return -1

    # TODO:
    #  - get list of avaliable files from repo
    #  - parse names to auto detect version from name

    if panther_version == "1.05":
        print("Detected version 1.05")
        return '/panther_hat_settings_v1.05.txt'
    elif panther_version == "1.20":
        print("Detected version 1.20")
        return '/panther_hat_settings_v1.20.txt'

def main():
    if not isinstance(sys.argv[1], str):
        print("wrong YAML file passed")
        return 1

    config_dir = sys.argv[1]
    yaml_file = config_dir + '/panther_config.yaml'

    with open(yaml_file, "r") as stream:
        try:
            config_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return 1

    out = get_hat_txt_config(config_file)

    # Generate files
    subprocess.call("cd /usr/local/bin && sudo ./eepmake {in_txt} {out_eep} -c {in_yaml}".format(
            in_txt = config_dir + out,
            out_eep = "/tmp/panther.eep",
            in_yaml = yaml_file) ,shell=True)


if __name__ == "__main__":
    main()

