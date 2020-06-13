#!/usr/bin/python

import os
import requests
import mmap

buildpath = "build"
arch_rootfs = "build/arch_rootfs"
sharepath = "build/share"
nfs4sharepath = "/srv/nfs4/ohCaptain"
if not os.path.exists(buildpath):
    os.mkdir(buildpath, 0o755)

if not os.path.exists(arch_rootfs):
    url_arch = "http://os.archlinuxarm.org/os/ArchLinuxARM-am33x-latest.tar.gz"
    url_boost = "http://mirror.archlinuxarm.org/armv7h/extra/boost-1.60.0-5-armv7h.pkg.tar.xz"
    filename_arch ="./build/ArchLinuxARM-am33x-latest.tar.gz"
    filename_boost = "./build/boost-1.60.0-5-armv7h.pkg.tar.xz"
    os.mkdir(arch_rootfs, 0o755)
    if not os.path.isfile(filename_arch):
        r = requests.get(url_arch)
        with open(filename_arch, "wb") as code:
            code.write(r.content)
        os.system("bsdtar -xpf " + filename_arch + " -C " + arch_rootfs)
        os.remove(filename_arch)

        r = requests.get(url_boost)
        with open(filename_boost, "wb") as code:
            code.write(r.content)
        os.system("bsdtar -xpf " + filename_boost + " -C " + arch_rootfs)
        os.remove(filename_boost)

if not os.path.exists(sharepath):
    if not os.path.exists(nfs4sharepath):
        os.mkdir(nfs4sharepath, 0o755)
    os.mkdir(sharepath, 0o755)
    os.system("mount --bind " + sharepath + " " + nfs4sharepath)
    os.system("exportfs -rav")
    f = open("/etc/fstab")
    s = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    if s.find(nfs4sharepath) == -1:
        with open("etc/fstab", "a") as fstabFile:
            fstabFile.write(os.getcwd() + sharepath + " " + nfs4sharepath + " none bind 0  0")