#!/usr/bin/python

import os
import sys
import requests
import tarfile

buildpath = "./build"
if not os.path.exists(buildpath):
    os.mkdir(buildpath, 0o755)

if not os.path.exists("./build/gcc-linaro"):
    url = "https://releases.linaro.org/components/toolchain/binaries/5.3-2016.02/arm-linux-gnueabihf/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf.tar.xz"
    filename ="./build/gcc-linaro.tar.xz"
    if not os.path.isfile(filename):
        r = requests.get(url)
        with open(filename, "wb") as code:
            code.write(r.content)
        tar = tarfile.open(filename)
        tar.extractall(buildpath)
        tar.close()
        os.remove(filename)
        os.rename("./build/gcc-linaro-5.3-2016.02-x86_64_arm-linux-gnueabihf", "./build/gcc-linaro")