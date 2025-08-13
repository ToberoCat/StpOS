# StpOS for debian Bookworm

> This is a guide to install StpOS on a Raspberry Pi 3 B+ with a touch display.
> The commands are designed to work on Ubuntu 24.04 LTS.
> This will most likely not work on windows, as windows cant access the sd cards root partition.
> You will need docker installed and running to complete this

## Create the initial sd card

Use the raspberry pi imager tool to create the initial sd card. Use the raspberry pi os lite 64 bit version. Configure
the hostname, user and password. Setup the wifi, locale and enable ssh.
Use debian bookworm 64 bit version.

### First time start

Now start the pi with the sd card - The first time start will take ~ 5 minutes with a few reboots. The display will have
scan lines - But don't worry, they'll get fixed in the next step.

### Run `setup-image.py`

To transfer all relevant files, run the `setup-image.py` script. This will copy the files needed files in the correct
folders automatically.

```bash
python3 setup-image.py --boot_partition <Boot Partition> --root_partition <Root Partition> --default_user <Default User>
```

Example (Linux)

```bash
python3 setup-image.py --boot_partition /media/tobias/bootfs --root_partition /media/tobias/rootfs --default_user pi
```

This copies the files `config.txt`, `cmdline.txt` and `tsc2007-overlay.dts` to the boot partition.


## Setup touch display (Dev Machine)

## Clone Repositories

The Raspberry Pi Linux repository is fairly large and may take some time to download

```bash
git clone --depth=1 https://github.com/raspberrypi/linux
git clone https://github.com/kipr/wombat-os
```

## Build Kernel

```bash
cd linux
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig
```

### Add Config Files

```bash
# Define mount point variables
MNT_BOOT="/media/tobias/bootfs"
MNT_ROOT="/media/tobias/rootfs"
```

```bash
`sed -i 's/# CONFIG_TOUCHSCREEN_TSC2007 is not set/CONFIG_TOUCHSCREEN_TSC2007=m/' .config
sudo cp assets/tsc2007-overlay.dts linux/arch/arm64/boot/dts/overlays/tsc2007-overlay.dts
sudo cp assets/Makefile linux/arch/arm64/boot/dts/overlays/Makefile`
```

### Copy Files to Boot and Root

Make sure your mount directories are actually mmcblk0p1 and not something else

```bash
sudo chmod 777 $MNT_ROOT/etc/modules
sudo echo 'tsc2007' >> $MNT_ROOT/etc/modules
```

Note: Depending on your machine, you may want to up this to higher than j12 for this build (j12 - 12 is the core count
you want to use for compiling)

```bash
cd linux

# Build dtbs
sudo make -j12 ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image dtbs

# Copy device tree files
sudo cp arch/arm64/boot/dts/broadcom/*.dtb $MNT_BOOT/
sudo cp arch/arm64/boot/dts/overlays/*.dtb* $MNT_BOOT/overlays/
sudo cp arch/arm/boot/dts/overlays/README $MNT_BOOT/overlays/
```

# Start

Now you can start the pi and connect via ssh. The display should be fine now.

# On Device setup

Now oyu have to configure the device's stm32.

## Install stm32flash

```bash
sudo apt update
sudo apt install stm32flash
```

## Enable serial ports

`sudo raspi-config` -> Interface Options -> Serial Port
login shell accessible over serial? -> **NO**
serial port hardware to be enabled? -> **YES**

## Flash Device

To flash, first clone the firmware repo:

```bash
git clone https://github.com/htl-stp-ecer/Firmware
```

Then, in the repo, modify the `deploy.sh` to point to the right ip and user. Then run it

```bash
cd Firmware
chmod +x deploy.sh
./deploy.sh
```

This will compile the latest version of the firmware and automatically flash it to the device.

# Install python

```bash
sudo apt-get install python3 python3-pip
```

#### DEPRECATED

The following steps are deprecated and will soon be obsolete. The kipr library will soon not be used anymore:

### Install libwallaby

Steps not listed, as soon obsolte.


# Set up the UI

## Building the UI and deploying it to the pi

Clone the ui from the stp github organistion.

The app must be built on your development machine. Note that you can't use a Raspberry Pi as your development machine.

### Host machine setup

1. Make sure you've installed the flutter SDK. Only flutter SDK >= 3.10.5 is supported for the new method at the moment.
2. Install the [flutterpi_tool](https://pub.dev/packages/flutterpi_tool):
   Run `flutter pub global activate flutterpi_tool` (One time only)
3. If running `flutterpi_tool` directly doesn't work,
   follow https://dart.dev/tools/pub/cmd/pub-global#running-a-script-from-your-path
   to add the dart global bin directory to your path.  
   Alternatively, you can launch the tool via:
   `flutter pub global run flutterpi_tool ...`

## Troubleshooting

I had the issue of getting an `Artifact not found` error. It was caused by me using a too new flutter version - No pre
build engine was found.

Fixed it by switching ti `3.24.4` (At this time, 3.24.5 was the latest version) because 3.24.4 had a engine prebuild
Engine version can be found here: https://github.com/ardera/flutter-engine-binaries-for-arm

## Setting up flutter pi on the raspberry itself

### Configuring your Raspberry Pi

> Note: This can be skipped in most cases, I just put it there in case there are issues

1. Open raspi-config:
    ```bash
    sudo raspi-config
    ```

2. Switch to console mode:
   `System Options -> Boot / Auto Login` and select `Console` or `Console (Autologin)`.

3. *You can skip this if you're on Raspberry Pi 4 with Raspbian Bullseye*  
   Enable the V3D graphics driver:  
   `Advanced Options -> GL Driver -> GL (Fake KMS)`

4. Configure the GPU memory
   `Performance Options -> GPU Memory` and enter `64`.

5. Leave `raspi-config`.

6. Give the `pi` permission to use 3D acceleration. (**NOTE:** potential security hazard. If you don't want to do this,
   launch `flutter-pi` using `sudo` instead.)
    ```bash
    sudo usermod -a -G render pi
    ```

7. Finish and reboot.

On the pi, install the flutter pi binaries:

These are the dependencies needed for building:

```bash
sudo apt install git cmake libgl1-mesa-dev libgles2-mesa-dev libegl1-mesa-dev libdrm-dev libgbm-dev ttf-mscorefonts-installer fontconfig libsystemd-dev libinput-dev libudev-dev  libxkbcommon-dev
sudo fc-cache
```

Build flutter pi. To clone the repo, you must have access to the htl stp ecer github account. If you don't have access,
you can still use the original project, but this will remove the ability to read sensor data. Add your ssh key to the
account to clone the repo.:

```bash
cd ~
git clone --recursive git@github.com:htl-stp-ecer/flutter-pi.git
cd flutter-pi
```

Compile the binary:

```bash
mkdir build && cd build
cmake ..
make -j`nproc`
```

Install the binary:

```bash
sudo make install
```

Remove flutter-pi folder to save disk space:

```bash
 rm -rf ~/flutter-pi/
```

### Update libinput to latest version

```bash
sudo apt install meson ninja-build libevdev-dev libwacom-dev libmtdev-dev libudev-dev libinput-dev libsystemd-dev libgtk-3-dev 
```

```bash
wget https://gitlab.freedesktop.org/libinput/libinput/-/archive/1.27.0/libinput-1.27.0.tar.gz
tar -xzf libinput-1.27.0.tar.gz
cd libinput-1.27.0
```

```bash
meson setup builddir
ninja -C builddir
sudo ninja -C builddir install
```

Check if it's the newest version:

```bash
libinput --version
```

Might need a reboot / new ssh session to take effect

Create the service:

```bash
sudo nano /etc/systemd/system/flutter-ui.service
```

Paste this content:

```bash
[Unit]
Description=Flutter UI with flutter-pi
After=network.target

[Service]
ExecStart=flutter-pi --videomode 800x480 --release /home/pi/stp-velox/
WorkingDirectory=/home/pi
User=pi
Group=pi
Restart=always
RestartSec=5
StandardOutput=tty
StandardError=tty

[Install]
WantedBy=multi-user.target
```

Enable & Start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable flutter-ui.service
sudo systemctl start flutter-ui.service
```

### Uploading StpVelox

CLone the stp velox repo to your local dev machine:

```bash
git clone git@github.com:htl-stp-ecer/StpVelox.git
```

In the repo, modify the `deploy.sh` and run it, to upload it to the pi. 

### Enabling the ui network manager

> Make sure you test the ui before running this, as doing so, you'll lose the ability to ssh into the pi, until you
> reconfigure the network.


For the ui to fully work, you must enable the network manager service. This can be done by running:

```bash
sudo systemctl enable NetworkManager
sudo systemctl start NetworkManager
```

Now, the wifi can be managed from the UI.

When everything works as expected, you should see the flutter pi UI on the display.
