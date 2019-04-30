#!/usr/bin/env python

import os
import shutil
import subprocess
import argparse

parser = argparse.ArgumentParser(description='Build the morepork kernel.')
parser.add_argument('-r', '--revision', type=str, default='macho-man',
                    help='electronics revision: stardust-reva or [macho-man]')
parser.add_argument('--dt-only', action='store_true',
                    help='Only build and install the device tree(s)')
parser.add_argument('--no-sgx', action='store_true',
                    help='Skip building the sgx kernel modules')
parser.add_argument('--no-modules', action='store_true',
                    help='Skip building/installing all modules')
parser.add_argument('--menuconfig', '-m', action='store_true',
                    help='Run menuconfig instead of building')
args = parser.parse_args()

if args.revision == 'macho-man':
    config = 'mb_macho_man_reva_defconfig'
    dts = ['mb-macho-man-reva.dtb', 'mb-macho-man-revd.dtb']
elif args.revision == 'stardust-reva':
    config = 'mb_stardust_reva_defconfig'
    dts = ['mb-stardust-reva.dtb']
else:
    print("Unrecognized electroncis revision %s" % args.revision)
    exit(-1)

this_dir = os.path.realpath(os.path.dirname(__file__))

# Install the modules to a staging path using make so that we
# can get a manifest of all files to actually install
staging_path = os.path.join(this_dir, 'obj/staging')

# Default value for debugging purposes -- gets overridden by TR
install_dir = os.path.abspath(os.path.join(this_dir, '../../Install'))

def run_cmd(cmd, cwd=this_dir):
    print(' '.join(cmd))
    subprocess.check_call(cmd, cwd=cwd)

def run_make_cmd(action, num_cores, cwd=this_dir):
    """
    run make with the given action. action should be a list of arguments,
    as you would send to subprocess, that will be appended to the make
    invocation. this function handles setting the cross-compilation options.
    """
    cross_compile = os.path.abspath(os.path.join(
        install_dir,
        'linaro-linux-gnu',
        'bin',
        'arm-linux-gnueabihf-'
    ))
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
        'INSTALL_MOD_PATH=' + staging_path,
    ]
    run_cmd(make_prefix + action, cwd=cwd)

def menuconfig():
    run_make_cmd([config], 1)
    run_make_cmd(['menuconfig'], 1)
    run_make_cmd(['savedefconfig'], 1)
    defconfig_src = os.path.join(this_dir, 'obj/defconfig')
    defconfig_tgt = os.path.join(this_dir, 'arch/arm/configs', config)
    shutil.copy2(defconfig_src, defconfig_tgt)

def build_sgx(num_cores=4):
    sgx_path = os.path.join(install_dir, 'sgx')
    sgx_make_opts = [
        'BUILD=release',
        'TI_PLATFORM=ti43xx',
        'OMAPES=9.x',
        'SUPPORT_XORG=0',
        'KERNELDIR=' + this_dir,
    ]

    # Despite passing in O= we still seem to be polluting our sgx
    # source tree so we copy it in clean on every build.
    # TODO: Every build is a clean build with this setup
    sgx_staging_path = os.path.join(this_dir, 'obj/sgx')
    if os.path.exists(sgx_staging_path):
        shutil.rmtree(sgx_staging_path)
    shutil.copytree(sgx_path, sgx_staging_path)

    run_make_cmd(sgx_make_opts + ['all'], num_cores, cwd=sgx_staging_path)
    run_make_cmd(sgx_make_opts + ['install'], 1, cwd=sgx_staging_path)

def build(num_cores=4):
    run_make_cmd([config], num_cores)
    for dt in dts:
        run_make_cmd([dt], num_cores)
    if args.dt_only: return
    run_make_cmd(['zImage'], num_cores)
    if os.path.exists(staging_path):
        shutil.rmtree(staging_path)
    # We need to create an artifact with a /lib -> /usr/lib symlink
    os.makedirs(staging_path + '/usr/lib')
    os.symlink('usr/lib', staging_path + '/lib')
    if args.no_modules: return
    run_make_cmd(['modules'], num_cores)
    run_make_cmd(['modules_install'], 1)
    if not args.no_sgx:
        build_sgx(num_cores)

def install_tree(src_path, install_path, manifest_file):
    """
    Recursively install the folder src_path as the folder install_path

    This will write the full install path of every file, symlink, and
    empty directory to the open file object manifest_file, mimicing
    the behavior of cmake when writing install_manifest.txt.  The
    copying behavior mimics "cp -ar" as closely as is possible without
    root permissions.
    """
    print("Installing %s as %s" % (src_path, install_path))
    for dir, subdirs, files in os.walk(src_path):
        rel_dir = os.path.relpath(dir, src_path)
        install_dir = os.path.abspath(os.path.join(install_path, rel_dir))
        if not os.path.exists(install_dir):
            os.makedirs(install_dir)
            shutil.copystat(dir, install_dir)
        if not files and not subdirs:
            manifest_file.write(install_dir + '\n')
        for file in files + subdirs:
            src_file = os.path.join(dir, file)
            install_file = os.path.join(install_dir, file)
            if os.path.islink(src_file):
                linkto = os.readlink(src_file)
                if os.path.lexists(install_file):
                    os.remove(install_file)
                os.symlink(linkto, install_file)
            elif os.path.isfile(src_file):
                if os.path.lexists(install_file):
                    os.remove(install_file)
                shutil.copy2(src_file, install_file)
            elif os.path.isdir(src_file):
                continue
            else:
                raise Exception("Unhandled file type")
            manifest_file.write(install_file + '\n')

def install_file(src_path, install_path, manifest_file):
    """
    Install a single regular file

    Usage is otherwise the same as install_tree
    """
    print("Installing %s as %s" % (src_path, install_path))
    install_dir = os.path.dirname(install_path)
    if not os.path.exists(install_dir):
        os.makedirs(install_dir)
    if os.path.lexists(install_path):
        os.remove(install_path)
    shutil.copy2(src_path, install_path)
    manifest_file.write(install_path + '\n')

def install(path):
    install_path = os.path.join(path, 'rootfs')
    install_boot = os.path.join(install_path, 'boot')
    image_path = os.path.join(this_dir, 'obj/arch/arm/boot')
    with open(os.path.join(this_dir, "install_manifest.txt"), 'w') as f:
        for dt in dts:
            install_file(
                os.path.join(image_path, 'dts', dt),
                os.path.join(install_boot, dt), f)
        if args.dt_only: return
        install_file(
            os.path.join(image_path, 'zImage'),
            os.path.join(install_boot, 'zImage'), f)
        if args.no_modules: return
        install_tree(staging_path, install_path, f)

def clean():
    run_make_cmd(['mrproper'], 1)

if 'TR_BUILD' in os.environ:
    # Convert boolean args back to booleans
    for arg in ('clean', 'docs', 'build', 'test', 'install'):
        globals()['TR_ARG_'+arg] = (os.environ['TR_ARG_'+arg] == 'True')

    install_dir = os.environ['TR_ARG_install_dir']

    if TR_ARG_clean:
        clean()

    if TR_ARG_build or TR_ARG_install:
        build(os.environ['TR_ARG_num_cores'])

    if TR_ARG_install:
        install(install_dir)

elif args.menuconfig:
    menuconfig()
else:
    build()
