#!/usr/bin/python3


import yaml

## RAW

def read_file(path):
    with open(path, 'r') as file:
        data = file.read().rstrip()

    return int(data)

## SMART

# def parse_yaml_to_list(d, keys=()):
#     if type(d) == dict:
#          for k in d:
#             for rv in parse_yaml_to_list(d[k], keys + (k, )):
#                 yield rv
#     else:
#         yield (keys, d)


with open("measurements.yaml", "r") as stream:
    try:
        config_file = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)


for measurement_id, data in config_file.items():
    path = data["path"]
    raw_value = read_file(path)
    print()
    print(raw_value)
    value = raw_value * data["scale"] * data["LSB"]

    print(measurement_id, "=",value,data["unit"])