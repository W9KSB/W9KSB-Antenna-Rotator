
# 🚀 Introducing the W9KSB Antenna Rotator

## About This Project (Overview)

This project is not a polished commercial product. It is a real-world, homebrew build that grew out of practical needs, experimentation, and a lot of tinkering.

The W9KSB Antenna Rotator exists because I wanted a reliable way to automate satellite tracking and reception from home without expensive commercial hardware. Along the way, the project evolved through trial, error, discussion, and learning — exactly how most good DIY projects do.

Everything here is shared **as-is**.  
There are **no warranties, guarantees, or promises of fitness** for any particular purpose.  
Hardware, software, and configurations may change over time, and what works in my environment may require adjustment in yours.

This GitHub repository hosts the bulk of the project, while my website provides detailed narrative posts, explanations, photos, and updates:

👉 https://www.w9ksb.com/

## Intent and Philosophy

- Document the build honestly — including decisions, tradeoffs, and limitations  
- Provide a clear starting point for others  
- Encourage experimentation, modification, and improvement  
- Share openly with the ham radio, SDR, and maker communities  

This is an off‑hours hobby project.  
Support is not guaranteed, and responses may be slow.  
Pull requests and respectful discussion are always welcome.

*(Disclaimer: This all needs updates I’m sure and I will be making many more edits. This is the literal first iteration of all of this, but I welcome feedback!)*

![Main Rotator Image](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0636.jpeg)

Satellite passes are short. Winter is cold. Holding a heavy Yagi pointed at the sky for 12–15 minutes while trying to decode SSTV or weather images gets old fast.

This rotator was built to solve exactly that.

This project is a fully DIY AZ/EL antenna rotator powered by a Raspberry Pi 5, designed to integrate well with satellite reception and SSTV decoding workflows. The Pi runs everything — including SDR processing — so you can place the rotator outside, and the only thing entering the house is a short SMA cable.

Everything runs headless on a Pi 5 and integrates with:

- **Gpredict**
- **SatDump**
- **Hamlib rotctld**
- **RTL-SDR (optional)**

It can also run **SSTV decoding**, **NOAA/Meteor weather imaging**, and other tasks directly onboard.

---

# 🎯 Goals

- Fully automate satellite tracking
- Avoid holding a Yagi in freezing weather
- Decode signals while the rotator tracks passes automatically
- Keep the software modular and simple
- Use off-the-shelf parts wherever possible
- Document the entire build so others can replicate or improve it

---

# 🧰 Hardware Overview

## Core Components

- Raspberry Pi 5  
- Adafruit DC & Stepper Motor HAT  
- Two worm-gear DC motors (AZ + EL)  
- AS5600 absolute magnetic encoders  
- PETG 3D-printed body  
- Tripod mount  
- Optional RTL-SDR v4 for on-board reception  

### Wiring Diagram

![Wiring Diagram](https://www.w9ksb.com/wp-content/uploads/2026/01/Rotator-1024x824.jpg)

Encoders:
- One on hardware I²C bus 1  
- One on software I²C bus 3 via bit-banged GPIO  

Motors:
- Driven by Motor HAT  
- Powered by a linear 12V supply  
- Pi is powered by a buck converter  

---

# 🧬 Software Overview

The software consists of several Python modules:

| File | Purpose |
|------|---------|
| **hamlib_server.py** | TCP server compatible with Hamlib/rotctld |
| **controller.py** | Brain of system — control loop, limits, homing |
| **movement.py** | Motor control via Motor HAT |
| **position.py** | AS5600 encoder interface |
| **config.py** | Central configuration file |
| **rotator_cal.json** | Stores calibration offsets |
| **manual.py** | Manual testing & jogging tool |

---

# 🏗 Build Photos & Movement Video

## 🎥 Movement Demo

[![Movement Demo](https://img.youtube.com/vi/zZoIDCE3QLc/hqdefault.jpg)](https://www.youtube.com/watch?v=zZoIDCE3QLc)

---

## 📸 Build Photos

### Tripod Mount

![1](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0643.jpeg)
![2](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0642.jpeg)
![3](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0641-1019x1024.jpeg)

### Rotator Body

![Top](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0636.jpeg)

### Elevation Assembly

![EL Hub](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0637-1024x984.jpeg)
![EL Side](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0640-1-1024x952.jpeg)

### Bottom Access

![Bottom](https://www.w9ksb.com/wp-content/uploads/2026/01/IMG_0639-676x1024.jpeg)

---

# 🛒 Parts List

## Core Electronics

| Product | Qty | Price | Link |
|--------|-----|--------|------|
| Raspberry Pi 5 | 1 | $91.00 | https://amzn.to/4rhhwjt |
| Adafruit Motor HAT | 1 | $29.79 | https://amzn.to/4t51I58 |
| Pin headers | 1 | $3.99 | https://amzn.to/4sYME8Z |
| AS5600 Encoders (4pk) | 1 | $9.99 | https://amzn.to/46frJ7J |
| Jumper wires | 1 | $6.98 | https://amzn.to/3O4QMnJ |

## Motors & Mechanics

| Product | Qty | Price | Link |
|--------|-----|--------|------|
| Worm gear motors | 2 | — | — |
| Hyper Hub 1310 (6mm) | 2 | $7.99 | https://www.servocity.com/1310-series-hyper-hub-6mm-d-bore/ |
| Roller bearing | 1 | $7.98 | https://amzn.to/4r6E8D8 |
| Heat-set inserts | 1 | $15.99 | https://amzn.to/460NZ5c |
| M3 screws | 1 | $6.99 | https://amzn.to/4r5rLHe |

## Structural

| Product | Qty | Price | Link |
|--------|-----|--------|------|
| Tripod stand | 1 | $37.99 | https://amzn.to/4t0KGF3 |
| PETG filament | 2 | $14.43 | https://amzn.to/4bfPoIE |
| Baseball donut weight | 1–2 | $7.99 | https://amzn.to/4bmx7JI |

## Power & Wiring

| Product | Qty | Price | Link |
|--------|-----|--------|------|
| Linear PSU | 1 | $20 | https://amzn.to/49Xoov6 |
| DC pigtail | 1 | $8.99 | https://amzn.to/4bp9bpf |
| 12→5V buck | 1 | $9.59 | https://amzn.to/3ZuMDfl |
| 12V buck | 1 | $7.99 | https://amzn.to/49IAQjy |
| 28 AWG wire | 1 | $5.99 | https://amzn.to/3NwWkYa |

## Optional Onboard SDR

| Product | Qty | Price | Link |
|--------|-----|--------|------|
| RTL-SDR v4 | 1 | $39.99 | https://amzn.to/4k0bTU7 |
| SMA cable | 1 | $6.99 | https://amzn.to/45tG7sN |
| SMA bulkhead | 1 | $5.98 | https://amzn.to/49TQMhJ |

---

# 🧠 Software Architecture Deep Dive

## 📡 hamlib_server.py

- Implements TCP rotctld protocol  
- Talks to Gpredict & SatDump  
- On disconnect → returns AZ & EL to home  

## 🧮 controller.py

Handles:

- Control loop  
- Shortest-path AZ logic  
- Elevation limit enforcement  
- Homing  
- Arrival detection  

## ⚙ movement.py

- Motor HAT interface  
- Speed control  
- Direction handling  

## 🧭 position.py

- Reads AS5600 encoders  
- Bus 1 (hardware) + Bus 3 (bit‑banged)  
- Converts raw readings into degrees  

## 📝 config.py

- Motor speeds  
- Pins  
- I²C bus mappings  
- Limits  
- Thresholds  

## 🎯 rotator_cal.json

Stores offsets:

- **Elevation offset** is usually the only one you need  
- **Azimuth home** auto-sets at startup  
- AZ offset still supported for customization (v1 feature)  

## 🎛 manual.py

A simple tool for:

- Motor jogging  
- Testing direction  
- Verifying encoders  
- Manual positioning  

---

# 🏃 Running the Code

## Install prerequisites

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-pip git i2c-tools adafruit-circuitpython-motorkit
```

## Enable I²C

```bash
sudo raspi-config
```

Enable I²C → Reboot.

## Enable Bit-Banged I²C (Bus 3)

Edit:

```bash
sudo nano /boot/config.txt
```

Add:

```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=24,i2c_gpio_scl=25
```

Reboot.

---

## Clone the repo

```bash
git clone https://github.com/W9KSB/W9KSB-Antenna-Rotator.git
cd W9KSB-Antenna-Rotator
```

---

## Run manually

```bash
sudo python3 -u hamlib_server.py
```

---

## Install as system service

Create:

```bash
sudo nano /etc/systemd/system/antenna-rotator.service
```

Paste:

```ini
[Unit]
Description=W9KSB Antenna Rotator Server
After=network.target

[Service]
ExecStart=/usr/bin/python3 -u /home/pi/W9KSB-Antenna-Rotator/hamlib_server.py
WorkingDirectory=/home/pi/W9KSB-Antenna-Rotator
StandardOutput=inherit
StandardError=inherit
Restart=always
User=root

[Install]
WantedBy=multi-user.target
```

Enable:

```bash
sudo systemctl daemon-reload
sudo systemctl enable antenna-rotator.service
sudo systemctl start antenna-rotator.service
```
