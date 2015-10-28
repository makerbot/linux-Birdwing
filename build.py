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

def build(num_cores=4):
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
    run_cmd(make_prefix + [config])
    run_cmd(make_prefix + [dt])
    run_cmd(make_prefix + ['zImage'])
    run_cmd(make_prefix + ['modules'])

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
        pass

else:
    build()
