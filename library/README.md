# LibStp

LibStp is a comprehensive library designed to simplify programming robots for Botball using the Wombat platform. It
abstracts the complexities of robotics programming by providing a unified interface for sensors and motors, making
development easier and more efficient. This library is particularly tailored for the Botball competition and ensures
compatibility with both the Create and Wombat hardware.

## How It Works

LibStp provides:

- A common interface for sensors and motors, eliminating hardware-specific coding differences.
- Python and C++ libraries for advanced robotics applications.
- A robust framework for creating complex robot behaviors with ease.
- Device-independent backends that allow same-code deployment on different hardware.

## Folder Structure

- **`pylib`**: Python helper functions and classes for interacting with the native library.
- **`external`**: External dependencies required for the native library and Python binding.
- **`examples`**: Python examples demonstrating the use of the library.
- **`slides`**: Some slides to help you get started with the library.

## Installation

### Installation on Raspberry Pi

This library is not available on public package managers like PyPI, as it is tailored for specific robotics
applications. To install it, you can choose between two methods:

#### Installing from a Wheel File

Building and using a wheel file is faster and ensures compatibility. This requires Docker to build the wheel file.

1. Clone the repository and navigate to the LibStp folder:

   ```bash
   git clone git@github.com:htl-stp-ecer/LibStp.git LibStp
   cd LibStp
   ```

2. To compile and upload the library, use the `deploy.py` script. Ensure SSH and SCP are installed on your computer. Use
   the `ssh-copy-id` command to enable password-less login to the Wombat (or Raspberry Pi). Make sure to first install
   these libraries on your device to run the script.

   ```bash
    pip install docker paramiko scp
    ```

   ```bash
   python3 deploy.py -h
   ```

#### `deploy.py` Usage:

```text
usage: deploy.py [-h] [--create3] [--libwallaby] [--ip IP] [--username USERNAME]
                 [--remote-dir REMOTE_DIR] [--python-version PYTHON_VERSION]
                 [--mode {compile,upload,both}]

Compile and/or upload the wheel and binaries to a Raspberry Pi.

options:
  -h, --help            Show this help message and exit
  --create3             Enable BUILD_CREATE3
  --libwallaby          Enable BUILD_LIBWALLABY
  --ip IP               Set Raspberry Pi IP address (default: 192.168.68.3)
  --username USERNAME   Set username for Raspberry Pi (default: pi)
  --remote-dir REMOTE_DIR
                        Set remote directory on Raspberry Pi (default: /tmp)
  --python-version PYTHON_VERSION
                        Set Python version (default: 39)
  --mode {compile,upload,both}
                        Operation mode: compile, upload, or both (default: both)
```

For automated deployment, set the IP, username, remote directory, and Python version. The current builds use Python
version 39.

> Some hints: When compiling, I had to rerun the script a few times because of various seg faults that just
> disappeared - Tbt, Idk why, but it works ig.
>

## Native Library (LibStp Native)

The native component of LibStp is written in C++ to provide high performance and direct hardware interaction. It
implements the core functionality of the library and is required for the Python bindings.

### On-Device Dependencies

- **LibWallaby** (Automatically installed with `deploy.py`)
- **Create3** (Automatically installed with `deploy.py`)
- **LibCurl** (Requires manual installation; only for Create3)

### Development Setup

#### Prerequisites

- **CLion IDE**: Strongly recommended for building and developing the project.
- **Docker**: Required for building the project and generating wheel files.
- **Raspberry Pi 3b+ or Wombat**: Recommended for testing the library directly on the target hardware.

#### Setting Up for Development

1. Open the folder in CLion.
2. Use the provided `Compile with Container` configuration to:
    - Compile the code.
    - Generate a Python wheel file.
    - Deploy the wheel file to the Raspberry Pi and install it.
3. Utilize the `pylib` folder for Python helper functions and classes.
4. Refer to the `external` folder for required dependencies.
5. Explore the `examples` folder for ready-to-use Python scripts.

#### Testing Locally (Limited Support)

To test the library locally on an emulator or container:

1. Run the `Compile wheel only` configuration in CLion. This launches the `deploy.py` script in compile mode.
2. Install the generated wheel file on your local environment.
3. Use the provided Docker file (`docker/Dockerfile.wombat-python`) to emulate the Wombat environment and configure it
   as a Python interpreter in PyCharm.

#### Manual Testing on Target Device

1. Deploy the library to the Raspberry Pi using the `Compile and upload` configuration.
2. SSH into the Pi or use PyCharm's interpreter setup to run an example or your own code.

## Additional Information

Refer to the **`documentation`** folder for detailed guidance on hardware setup, library usage, and API references.
These resources provide all necessary information to maximize the potential of LibStp.