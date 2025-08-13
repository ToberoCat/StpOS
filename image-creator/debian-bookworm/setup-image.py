import os
import argparse
import logging
import shutil

mount_point: str = ""
root_mount_point: str = ""
default_user: str = ""


def copy_assets_to_sd_card(asset: str, destination: str):
    source_path = os.path.join("assets", asset)
    destination_path = os.path.join(mount_point, destination)
    logging.info(f"Copying {os.path.abspath(source_path)} to {os.path.abspath(destination_path)}")
    if not os.path.exists(source_path):
        logging.error(f"Asset {source_path} does not exist.")
        raise FileNotFoundError(f"Asset {source_path} not found.")
    shutil.copy2(source_path, destination_path)


def append_to_cmdline_txt():
    cmdline_path = os.path.join(mount_point, "cmdline.txt")
    logging.info(f"Appending 'video=HDMI-A-1:800x480M@60D' to {cmdline_path}")
    if not os.path.exists(cmdline_path):
        logging.error(f"{cmdline_path} does not exist.")
        raise FileNotFoundError(f"{cmdline_path} not found.")
    with open(cmdline_path, "a") as cmdline_file:
        cmdline_file.write(" video=HDMI-A-1:800x480M@60D\n")


def receive_mount_point(_mount_point):
    global mount_point
    if not os.path.ismount(_mount_point):
        raise ValueError(f"{_mount_point} is not a valid mount point.")
    logging.info(f"Mount point received: {_mount_point}")
    mount_point = _mount_point


def receive_partitions(boot_partition, root_partition, user):
    global mount_point, root_mount_point, default_user
    if not os.path.ismount(boot_partition):
        raise ValueError(f"{boot_partition} is not a valid mount point.")
    if not os.path.ismount(root_partition):
        raise ValueError(f"{root_partition} is not a valid mount point.")
    logging.info(f"Boot partition: {boot_partition}")
    logging.info(f"Root partition: {root_partition}")
    mount_point = boot_partition
    root_mount_point = root_partition
    default_user = user


def copy_flash_files():
    flash_files_dir = os.path.join(root_mount_point, "home", default_user, "flashFiles")
    os.makedirs(flash_files_dir, exist_ok=True)
    for asset_name in ["wallaby_flash", "wallaby_init_gpio", "wallaby_reset_coproc"]:
        source_path = os.path.join("assets", "flashFiles", asset_name)
        dest_path = os.path.join(flash_files_dir, asset_name)
        logging.info(f"Copying {source_path} to {dest_path}")
        shutil.copy2(source_path, dest_path)


def copy_resources_to_sd_card():
    logging.info("Copying resources to the SD card.")
    copy_assets_to_sd_card("config.txt", "config.txt")
    append_to_cmdline_txt()
    copy_flash_files()


def main():
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(description="CLI Tool for setting up the SD card.")
    parser.add_argument('--boot_partition', type=str, required=True, help='Mount point for the boot partition')
    parser.add_argument('--root_partition', type=str, required=True, help='Mount point for the root partition')
    parser.add_argument('--default_user', type=str, required=True, help='Default user name')

    args = parser.parse_args()
    receive_partitions(args.boot_partition, args.root_partition, args.default_user)
    logging.info("Setting up the SD card. Make sure ")
    
    copy_resources_to_sd_card()

    logging.info("Setup complete.")


if __name__ == "__main__":
    main()
