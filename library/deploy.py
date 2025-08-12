#!/usr/bin/env python3

import argparse
import logging
import os
import re
import subprocess
import sys
import zipfile

from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient

# Define constant versions for kipr
KIPR_VERSION = "1.2.0"
LIBSTP_VERSION = "1.0.0"

build_flags = {
    "libwallaby": False,
}

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


def should_build_libwallaby(args):
    return build_flags["libwallaby"] or args.libwallaby


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Compile and/or upload the wheel and binaries to a Raspberry Pi."
    )
    parser.add_argument(
        "--libwallaby",
        action="store_true",
        help="Enable BUILD_LIBWALLABY",
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable DEBUG_BUILD",
    )
    parser.add_argument(
        "--ip",
        type=str,
        default="192.168.68.3",
        help="Set Raspberry Pi IP address (default: 192.168.68.3)",
    )
    parser.add_argument(
        "--username",
        type=str,
        default="pi",
        help="Set username for Raspberry Pi (default: pi)",
    )
    parser.add_argument(
        "--remote-dir",
        type=str,
        default="/tmp",
        help="Set remote directory on Raspberry Pi (default: /tmp)",
    )
    parser.add_argument(
        "--python-version",
        type=str,
        default="311",
        help="Set Python version (default: 311)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["compile", "upload", "docs", "all"],
        default="all",
        help="Operation mode: compile, upload, docs or all (default: all)",
    )
    return parser.parse_args()


def run_subprocess(command, cwd=None):
    logging.info(f"Running command: {' '.join(command)}")
    try:
        subprocess.run(command, check=True, cwd=cwd)
    except subprocess.CalledProcessError as e:
        logging.error(f"Command failed with exit code {e.returncode}")
        sys.exit(e.returncode)


def ensure_binaries(args):
    """
    Ensure that the ./binaries folder exists and contains the required .deb files.
    If any .deb file is missing, attempt to build it.
    """
    binaries_dir = os.path.join(os.getcwd(), "binaries")
    os.makedirs(binaries_dir, exist_ok=True)
    logging.info(f"Ensured that binaries directory exists at '{binaries_dir}'.")

    # Define expected binary files with regex patterns
    binaries = {
        "libwallaby": rf"kipr-{KIPR_VERSION}-Linux\.deb",
    }

    for key, file_regex in binaries.items():
        files = os.listdir(binaries_dir)
        is_file_present = any([re.match(file_regex, file) for file in files])
        if not is_file_present:
            logging.warning(f"Missing binary: {file_regex}")
            logging.info(f"Additionally building {file_regex}...")
            if key in build_flags:
                build_flags[key] = True
            else:
                logging.warning(f"{file_regex} is not required for this deployment mode.")
        else:
            logging.info(f"Found binary: {file_regex}")

    return


def compile_wheel(args):
    logging.info("Starting compilation process...")

    # Ensure binaries are present or built
    ensure_binaries(args)

    # Initialize Docker client
    try:
        run_subprocess(["docker", "info"])
    except Exception as e:
        logging.error(f"Docker is not running or not accessible: {e}")
        sys.exit(1)

    builder_name = "multiarch_builder"
    image_name = "libstp-builder:python3.11"

    # Check if the builder exists using subprocess
    logging.info(f"Checking if Docker buildx builder '{builder_name}' exists...")
    try:
        result = subprocess.run(
            ["docker", "buildx", "ls", "--format", "{{.Name}}"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        builders = result.stdout.strip().split('\n')
        builder_exists = builder_name in builders
    except subprocess.CalledProcessError as e:
        logging.error(f"Failed to list Docker buildx builders: {e.stderr}")
        sys.exit(e.returncode)

    if not builder_exists:
        logging.info(f"Creating Docker buildx builder '{builder_name}'...")
        run_subprocess(["docker", "buildx", "create", "--name", builder_name, "--use"])
    else:
        logging.info(f"Using existing Docker buildx builder '{builder_name}'...")
        run_subprocess(["docker", "buildx", "use", builder_name])

    # Inspect and bootstrap the builder
    logging.info(f"Inspecting and bootstrapping builder '{builder_name}'...")
    run_subprocess(["docker", "buildx", "inspect", builder_name, "--bootstrap"])

    # Register qemu
    logging.info("Registering qemu for multi-architecture builds...")
    run_subprocess([
        "docker", "run", "--rm", "--privileged",
        "multiarch/qemu-user-static", "--reset", "-p", "yes"
    ])

    # Check if Docker image exists using subprocess
    logging.info(f"Checking if Docker image '{image_name}' exists locally...")
    try:
        subprocess.run(
            ["docker", "image", "inspect", image_name],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        image_exists = True
    except subprocess.CalledProcessError:
        image_exists = False

    if not image_exists:
        logging.info(f"Docker image '{image_name}' not found. Building the image...")
        run_subprocess([
            "docker", "buildx", "build",
            "--platform", "linux/arm64",
            "-t", image_name,
            "--load",
            "."
        ])
    else:
        logging.info(f"Docker image '{image_name}' found. Skipping build.")

    # Run Docker container to build the wheel
    logging.info("Running Docker container to build the wheel...")
    build_libwallaby_flag = "--libwallaby" if should_build_libwallaby(args) else ""
    build_in_debug_mode = "--debug" if args.debug else ""
    if build_in_debug_mode:
        logging.info("Building in debug mode...")

    run_subprocess([
        "docker", "run", "--rm", "--privileged",
        "--platform=linux/arm64",
        f"--cpus={os.cpu_count()}",
        f"--memory=16G",
        "-v", f"{os.getcwd()}:/app",
        image_name,
        "bash", "-c",
        f"bash ./scripts/build-in-container.sh {build_libwallaby_flag} {build_in_debug_mode}"
    ])

    logging.info("Compilation process completed successfully.")


def install_wheel(args):
    logging.info("Starting upload process...")

    # Define variables
    RASPBERRY_PI_IP = args.ip
    USERNAME = args.username
    REMOTE_DIR = args.remote_dir
    PYTHON_VERSION = args.python_version
    WHEEL_FILE_NAME = f"libstp-{LIBSTP_VERSION}-cp{PYTHON_VERSION}-cp{PYTHON_VERSION}-linux_aarch64.whl"

    LIBWALLABY = args.libwallaby

    binaries_dir = os.path.join(os.getcwd(), "binaries")
    kipr_deb = os.path.join(binaries_dir, f"kipr-{KIPR_VERSION}-Linux.deb")

    # Establish SSH connection
    logging.info(f"Connecting to Raspberry Pi at {RASPBERRY_PI_IP}...")
    ssh = SSHClient()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    try:
        ssh.connect(RASPBERRY_PI_IP, username=USERNAME)
    except Exception as e:
        logging.error(f"SSH connection failed: {e}")
        sys.exit(1)

    scp = SCPClient(ssh.get_transport())

    # Upload and install libwallaby if required
    if LIBWALLABY and os.path.isfile(kipr_deb):
        logging.info("Uploading and installing libwallaby package...")
        try:
            scp.put(kipr_deb, f"{REMOTE_DIR}/kipr-{KIPR_VERSION}-Linux.deb")
            stdin, stdout, stderr = ssh.exec_command(f"sudo dpkg -i {REMOTE_DIR}/kipr-{KIPR_VERSION}-Linux.deb")
            exit_status = stdout.channel.recv_exit_status()
            if exit_status != 0:
                logging.error(f"Error installing libwallaby: {stderr.read().decode()}")
                sys.exit(exit_status)
        except Exception as e:
            logging.error(f"Failed to upload/install libwallaby: {e}")
            sys.exit(1)
    elif LIBWALLABY:
        logging.error(f"libwallaby binary not found at '{kipr_deb}'. Please ensure it is built.")
        sys.exit(1)

    # Upload and install the wheel file
    wheel_path = os.path.join("dist", WHEEL_FILE_NAME)
    if not os.path.isfile(wheel_path):
        logging.error(f"Wheel file '{wheel_path}' does not exist. Please compile first.")
        sys.exit(1)

    logging.info(f"Uploading wheel file '{WHEEL_FILE_NAME}'...")
    try:
        scp.put(wheel_path, f"{REMOTE_DIR}/{WHEEL_FILE_NAME}")
        stdin, stdout, stderr = ssh.exec_command(f"pip install --force-reinstall {REMOTE_DIR}/{WHEEL_FILE_NAME} --break-system-packages")
        exit_status = stdout.channel.recv_exit_status()
        if exit_status != 0:
            logging.error(f"Error installing wheel: {stderr.read().decode()}")
            sys.exit(exit_status)
    except Exception as e:
        logging.error(f"Failed to upload/install wheel: {e}")
        sys.exit(1)

    # Close connections
    scp.close()
    ssh.close()

    logging.info("Upload process completed successfully.")


def generate_documentation():
    logging.info("Generating documentation...")
    docs_dir = os.path.join(os.getcwd(), "docs")
    logging.info("Setting up Docker environment for documentation generation...")

    try:
        run_subprocess(["docker", "buildx", "create", "--name", "libstp_builder", "--use"])
    except SystemExit:
        logging.warning("Builder 'libstp_builder' might already exist. Attempting to use the existing builder.")
        run_subprocess(["docker", "buildx", "use", "libstp_builder"])

    run_subprocess(["docker", "buildx", "inspect", "--bootstrap"])

    image_name = "libstp_emulated"
    dockerfile_path = os.path.join("docker", "Dockerfile.wombat-python")
    run_subprocess([
        "docker", "buildx", "build",
        "--platform", "linux/arm64",
        "-t", image_name,
        "-f", dockerfile_path,
        "--load",
        "."
    ])

    run_subprocess([
        "docker", "run", "--rm", "--privileged",
        "multiarch/qemu-user-static", "--reset", "-p", "yes"
    ])

    logging.info("Running Docker container to build the documentation...")
    # ToDo: Update the libray installed in the container to the latest version
    run_subprocess([
        "docker", "run", "--rm", "--privileged",
        "--platform=linux/arm64/v8",
        "-v", f"{docs_dir}:/docs",
        image_name,
        "bash", "-c", "cd /docs && make html"
    ])

    logging.info("Documentation generated successfully.")

def main():
    args = parse_arguments()

    if args.mode not in ["compile", "upload", "docs", "all"]:
        logging.error("Invalid mode. Choose from 'compile', 'upload', 'docs', or 'all'.")
        sys.exit(1)

    if args.mode in ["compile", "all"]:
        compile_wheel(args)

    if args.mode in ["upload", "all"]:
        install_wheel(args)

    if args.mode in ["docs", "all"]:
        generate_documentation()

    logging.info("Deployment process completed successfully.")
    logging.info("When everything worked, you'll now have the following files:")
    logging.info(f"  binaries/kipr-{KIPR_VERSION}-Linux.deb                              The libwallaby binary")
    logging.info("  dist/libstp-0.0.1-cp39-cp39-linux_aarch64.whl              The built & installable python library")
    logging.info("  docs/build/html/                                           The generated documentation")
    logging.info("Exiting...")

if __name__ == "__main__":
    main()
