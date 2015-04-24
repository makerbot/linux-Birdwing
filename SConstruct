import sys
import os
import fnmatch

env = Environment(ENV = os.environ)

#TODO: Use mw_scons_tools rather than copy/pasting this
# This is a special glob made by NicholasBishop
def mb_recursive_file_glob(env, root, pattern, exclude = None):
    """Recursively search in 'root' for files matching 'pattern'

    Returns a list of matches of type SCons.Node.FS.File.

    If exclude is not None, it should be a glob pattern or list of
    glob patterns. Any results matching a glob in exclude will be
    excluded from the returned list."""
    def excluded(filename, exclude):
        if exclude:
            if isinstance(exclude, str):
                exclude = [exclude]
            for pattern in exclude:
                if fnmatch.fnmatch(filename, pattern):
                    return True
        return False

    matches = []
    if root.startswith('#'):
        raise Exception('Directories starting with "#" not supported yet')
    project_root = env.Dir('#').abspath
    for parent, dirnames, filenames in os.walk(os.path.join(
            project_root, root)):
        for filename in fnmatch.filter(filenames, pattern):
            if not excluded(filename, exclude):
                p = os.path.join(parent, filename)
                rp = os.path.relpath(p, project_root)
                matches.append(env.File(rp))
    return matches

env.AddMethod(mb_recursive_file_glob, 'MBRecursiveFileGlob')

linuxDir = os.path.abspath(str(Dir('#')))
baseDir = os.path.abspath(os.path.join(linuxDir, os.pardir))

subenv = os.environ.copy()

# U-boot uses the angstrom toolchain
angstrom = os.path.join(baseDir, 'Birdwing-Cross-Compile-Tools', 'angstrom', 'arm')
tool_prefix = 'arm-angstrom-linux-gnueabi'
env.PrependENVPath('PATH', os.path.join(angstrom, 'bin'))
# I hope we don't actually need this, since it breaks builds on most systems
#env.PrependENVPath('CPATH', os.path.join(angstrom, tool_prefix, 'usr', 'include'))
env['ENV']['LIBTOOL_SYSROOT_PATH'] = os.path.join(angstrom, tool_prefix)
env['ENV']['PKG_CONFIG_SYSROOT_DIR'] = os.path.join(angstrom, tool_prefix)
env['ENV']['PKG_CONFIG_PATH'] = os.path.join(angstrom, tool_prefix, 'usr', 'lib', 'pkgconfig')
env['ENV']['CONFIG_SITE'] = os.path.join(angstrom, 'site-config')

# Yes, this is a scons script to call make
def make_cmd(*args):
    makebaseCmd = ['make',
        'ARCH=arm',
        'CROSS_COMPILE=%s-' % (tool_prefix),
    ]
    makeCmd = makebaseCmd + list(args)
    return ' '.join(makeCmd)

config_sources = [
    'arch/arm/configs/mb_manhattan_defconfig'
]

config_targets = [
    '.config',
    'scripts/basic/.fixdep.cmd',
    'scripts/basic/fixdep',
    'scripts/kconfig/.conf.cmd',
    'scripts/kconfig/.conf.o.cmd',
    'scripts/kconfig/.zconf.tab.o.cmd',
    'scripts/kconfig/conf',
    'scripts/kconfig/conf.o',
    'scripts/kconfig/zconf.hash.c',
    'scripts/kconfig/zconf.lex.c',
    'scripts/kconfig/zconf.tab.c',
    'scripts/kconfig/zconf.tab.o',
]

env.Command(config_targets, config_sources, make_cmd('mb_manhattan_defconfig'))

gen_sources = [
    'arch/arm/boot/compressed/ashldi3.S',
    'arch/arm/boot/compressed/lib1funcs.S',
    'arch/arm/kernel/asm-offsets.s',
    'drivers/tty/vt/consolemap_deftbl.c',
    'drivers/tty/vt/defkeymap.c',
    'kernel/bounds.s',
    'kernel/config_data.h',
    'kernel/timeconst.h',
    'lib/crc32table.h',
    'scripts/dtc/dtc-lexer.lex.c',
    'scripts/dtc/dtc-parser.tab.c',
    'scripts/dtc/dtc-parser.tab.h',
    'scripts/genksyms/keywords.hash.c',
    'scripts/genksyms/lex.lex.c',
    'scripts/genksyms/parse.tab.c',
    'scripts/genksyms/parse.tab.h',
    'scripts/mod/elfconfig.h',
]
gen_sources.extend(env.MBRecursiveFileGlob('drivers/video/logo', '*.c', 'logo.c'))

build_sources = []

build_targets = [
    'arch/arm/boot/uImage',
]

build = env.Command(build_targets, build_sources, make_cmd('uImage'))
AlwaysBuild(build) # Let make determine what needs to be built

clean_targets = [
    '.missing-syscalls.d',
    '.version',
    'Module.symvers',
    'System.map',
    'arch/arm/boot/Image',
    'arch/arm/boot/compressed/piggy.gzip',
    'arch/arm/boot/compressed/vmlinux',
    'arch/arm/boot/compressed/vmlinux.lds',
    'arch/arm/boot/zImage',
    'arch/arm/kernel/vmlinux.lds',
    'arch/arm/lib/lib.a',
    'include/config/',
    'include/generated/',
    'kernel/config_data.gz',
    'lib/gen_crc32table',
    'lib/lib.a',
    'scripts/bin2c',
    'scripts/conmakehash',
    'scripts/dtc/dtc',
    'scripts/genksyms/genksyms',
    'scripts/kallsyms',
    'scripts/mod/mk_elfconfig',
    'scripts/mod/modpost',
    'scripts/pnmtologo',
    'scripts/sortextable',
    'usr/.initramfs_data.cpio.d',
    'usr/gen_init_cpio',
    'usr/initramfs_data.cpio',
    'vmlinux',
]

clean_targets.extend(gen_sources)
clean_targets.extend(env.Glob('.tmp_*'))
clean_targets.extend(env.MBRecursiveFileGlob('.', '*.o'))
clean_targets.extend(env.MBRecursiveFileGlob('.', '.*.cmd'))
env.Clean(build, clean_targets)

