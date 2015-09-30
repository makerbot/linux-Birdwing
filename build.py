#!/usr/bin/env python

import os
import shutil
import subprocess

# Set the build configuration here as <name>_defconfig
config = 'mb_stardust_reva_defconfig'

# We also need to specify a device tree to build
dt = 'mb-stardust-reva.dtb'

this_dir = os.path.realpath(os.path.dirname(__file__))

def run_cmd(cmd):
    print(' '.join(cmd))
    subprocess.check_call(cmd, cwd=this_dir)

cross_compile = os.path.abspath(os.path.join(
    this_dir,
    os.pardir,
    'linaro-bare-metal',
    'gcc-linaro-4.9-2015.02-3-x86_64_arm-eabi',
    'bin',
    'arm-eabi-'
))

def run_make_cmd(action, num_cores):
    """
    run make with the given action. action should be a list of arguments,
    as you would send to subprocess, that will be appended to the make
    invocation. this function handles setting the cross-compilation options.
    """
    obj_dir = os.path.join(this_dir, 'obj')
    if not os.path.exists(obj_dir):
        os.makedirs(obj_dir)
    make_prefix = [
        'make',
        #'V=1',
        '-j', str(num_cores),
        'O=' + obj_dir,
        'ARCH=arm',
        'CROSS_COMPILE=' + cross_compile,
    ]
    run_cmd(make_prefix + action)

def build(num_cores=4):
    run_make_cmd([config], num_cores)
    run_make_cmd([dt], num_cores)
    run_make_cmd(['zImage'], num_cores)
    run_make_cmd(['modules'], num_cores)

def install(path):
    install_path = os.path.join(path, 'rootfs')
    install_boot = os.path.join(install_path, 'boot')
    if not os.path.exists(install_boot):
        os.makedirs(install_boot)
    run_make_cmd(['modules_install', 'INSTALL_MOD_PATH='+install_path], 1)
    image_path = os.path.join(this_dir, 'obj', 'arch', 'arm', 'boot')
    shutil.copy2(os.path.join(image_path, 'zImage'),
                 install_boot)
    shutil.copy2(os.path.join(image_path, 'dts', 'mb-stardust-reva.dtb'),
                 install_boot)

def clean():
    print("Removing obj/")
    shutil.rmtree(os.path.join(this_dir, 'obj'))

if 'TR_BUILD' in os.environ:
    # Convert boolean args back to booleans
    for arg in ('clean', 'docs', 'build', 'test', 'install'):
        globals()['TR_ARG_'+arg] = (os.environ['TR_ARG_'+arg] == 'True')

    if TR_ARG_clean:
        clean()

    if TR_ARG_build or TR_ARG_install:
        build(os.environ['TR_ARG_num_cores'])

    # TODO: figure out something for install
    if TR_ARG_install:
        install(os.environ['TR_ARG_install_dir'])

else:
    build()
