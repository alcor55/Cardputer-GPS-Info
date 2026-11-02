"""Package one complete ESP32-S3 image for M5Burner and M5Launcher."""

from pathlib import Path
import re
import subprocess

Import("env")


def package_firmware(source, target, env):
    project = Path(env.subst("$PROJECT_DIR"))
    sketch = (project / "Cardputer_GPS_Info.ino").read_text()
    version = re.search(r'^#define\s+APP_VERSION\s+"([^"]+)"', sketch, re.MULTILINE)
    if not version:
        raise RuntimeError("Cannot find APP_VERSION in the sketch")

    output = project / "build" / "release"
    output.mkdir(parents=True, exist_ok=True)
    firmware = output / ("Cardputer_GPS_Info-" + version.group(1) + "-universal.bin")
    images = list(env.get("FLASH_EXTRA_IMAGES", []))
    images.append((env.subst("$ESP32_APP_OFFSET"), str(target[0])))
    images.sort(key=lambda image: int(str(image[0]), 0))
    if not images or int(str(images[0][0]), 0) != 0:
        raise RuntimeError("The ESP32-S3 package must start with its bootloader at 0x0")

    command = [env.subst("$PYTHONEXE"), env.subst("$UPLOADER"),
        "--chip", "esp32s3", "merge_bin", "--flash_size",
        env.BoardConfig().get("upload.flash_size"), "-o", str(firmware)]
    for address, image in images:
        command.extend([str(address), env.subst(str(image))])
    subprocess.run(command, check=True)
    print("M5Burner (0x0) / M5Launcher: " + str(firmware))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", package_firmware)
