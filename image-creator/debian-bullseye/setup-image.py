import os
import argparse
import logging
import shutil

mount_point: str = ""


def copy_assets_to_sd_card(asset: str, destination: str):
    logging.info(f"Copying {asset} to {destination}")
    source_path = os.path.join("assets", asset)
    destination_path = os.path.join(mount_point, destination)
    if not os.path.exists(source_path):
        logging.error(f"Asset {source_path} does not exist.")
        raise FileNotFoundError(f"Asset {source_path} not found.")
    shutil.copy2(source_path, destination_path)


def receive_mount_point(_mount_point):
    global mount_point
    if not os.path.ismount(_mount_point):
        raise ValueError(f"{_mount_point} is not a valid mount point.")
    logging.info(f"Mount point received: {_mount_point}")
    mount_point = _mount_point


def copy_resources_to_sd_card():
    logging.info("Copying resources to the SD card.")
    copy_assets_to_sd_card("config.txt", "config.txt")
    copy_assets_to_sd_card("tsc2007-overlay.dts", os.path.join("overlays", "tsc2007-overlay.dts"))

def main():
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    parser = argparse.ArgumentParser(description="CLI Tool for setting up the SD card.")
    parser.add_argument('-m', '--mount_point', type=str, help='The mount point of the SD card', required=True)

    args = parser.parse_args()
    receive_mount_point(args.mount_point)
    logging.info("Setting up the SD card. Make sure ")
    
    copy_resources_to_sd_card()

    logging.info("Setup complete.")


if __name__ == "__main__":
    main()
