# Demos

## Launching tutorial

### 1. Launching on PC

Clone the repo

```bash
git clone https://github.com/husarion/panther_driver
```

Run rviz container

```bash
xhost local:root
docker compose -f compose.pc.yaml up
```

### 2. Launching on Panther

Connect to panther WIFI SSID: `Panther_<id>`

SSH to Main computer:

```bash
ssh husarion@10.15.20.3
```

Clone the repo

```bash
git clone https://github.com/husarion/panther_driver
```

Go to the `panther_driver/demo` folder:

```bash
cd panther_driver/demo
```

Launch demo:

```bash
docker compose -f compose.panther.yaml up
```