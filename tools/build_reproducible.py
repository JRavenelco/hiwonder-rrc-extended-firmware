#!/usr/bin/env python3
"""Reproducible build of the bench-validated firmware image.

Rebuilds the image with a stock Arm GNU Toolchain, outside any IDE, and checks
that the resulting Intel HEX is bit-for-bit the one that was flashed and
validated on the real board (see EXPECTED_HEX_SHA256 below).

Why a script when there is an STM32CubeIDE project next to it?  The IDE build
is the comfortable way to develop, but an IDE decides the compiler version and
the order in which objects are linked, and both change the bytes of the image.
This script pins the three things that make a build reproducible:

  * the exact list of sources and their link order   (tools/link-order.txt)
  * the include search order                         (tools/include-dirs.txt)
  * the compiler and linker options                  (CFLAGS / LDFLAGS below)

With Arm GNU Toolchain 15.2.Rel1 the output is identical on x86-64 and on
aarch64 hosts, and it does not depend on where the repository is checked out.

Usage:
    python3 tools/build_reproducible.py [--out DIR] [--jobs N]

    CC, OBJCOPY and SIZE can be overridden through the environment; by default
    arm-none-eabi-gcc, -objcopy and -size are taken from PATH.

Exit status: 0 if the HEX matches the validated image, 1 if it builds but
differs (for example with another compiler version), 2 on a build error.
"""
import argparse
import concurrent.futures
import hashlib
import os
import subprocess
import sys

# SHA-256 of RosRobotControllerM4-extended-m4-pos-v8.hex, the image validated
# on the bench on 2026-10-04 (position control on M4, 5.7 rev/s limit,
# independent watchdog, position HOLD).
EXPECTED_HEX_SHA256 = '3a4d9e885ac408b11da56e7263b9e94c843f1b7bbbbf399f21cc93ab4a23f1e8'
EXPECTED_TOOLCHAIN = 'Arm GNU Toolchain 15.2.Rel1'

MCU = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
CFLAGS = MCU + [
    '-DUSE_HAL_DRIVER', '-DSTM32F407xx',
    '-DLV_CONF_INCLUDE_SIMPLE', '-DLV_LVGL_H_INCLUDE_SIMPLE',
    '-ffunction-sections', '-fdata-sections',   # one section per symbol, so the linker can drop unused code
    '-fcommon', '-fno-strict-aliasing',         # part of the validated configuration (docs/BUILDING.md)
    '-Os', '-g3', '-Wall', '-Wextra', '-Wno-unused-parameter', '-Wno-sign-compare',
]
LDFLAGS = MCU + ['-Wl,--gc-sections', '--specs=nano.specs', '--specs=nosys.specs']
LIBS = ['-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group']


def read_list(path):
    with open(path, encoding='utf-8') as f:
        return [ln.strip() for ln in f if ln.strip() and not ln.startswith('#')]


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default=os.path.join(repo, 'build-reproducible'))
    ap.add_argument('--jobs', type=int, default=os.cpu_count() or 1)
    args = ap.parse_args()

    fw = os.path.join(repo, 'firmware')
    sources = read_list(os.path.join(repo, 'tools', 'link-order.txt'))
    includes = read_list(os.path.join(repo, 'tools', 'include-dirs.txt'))
    cc = os.environ.get('CC', 'arm-none-eabi-gcc')
    objcopy = os.environ.get('OBJCOPY', 'arm-none-eabi-objcopy')
    size = os.environ.get('SIZE', 'arm-none-eabi-size')

    try:
        version = subprocess.run([cc, '--version'], capture_output=True, text=True, check=True).stdout.splitlines()[0]
    except (OSError, subprocess.CalledProcessError):
        print(f'error: cannot run {cc}; put the Arm GNU Toolchain in PATH or set CC', file=sys.stderr)
        return 2
    print(f'compiler : {version}')
    if EXPECTED_TOOLCHAIN not in version:
        print(f'note     : the validated image was built with {EXPECTED_TOOLCHAIN}; '
              'another version is expected to produce different bytes')

    obj_dir = os.path.join(args.out, 'obj')
    os.makedirs(obj_dir, exist_ok=True)
    cflags = CFLAGS + ['-I' + os.path.join(fw, d) for d in includes]

    # Objects are numbered in link order. The number also keeps two sources
    # with the same file name in different folders from overwriting each other.
    jobs = []
    for i, rel in enumerate(sources, 1):
        src = os.path.join(fw, rel)
        obj = os.path.join(obj_dir, f'{i:03d}_{os.path.splitext(os.path.basename(rel))[0]}.o')
        lang = ['-std=gnu11'] if rel.lower().endswith('.c') else ['-x', 'assembler-with-cpp']
        jobs.append((rel, [cc] + cflags + lang + ['-c', src, '-o', obj], obj))

    def compile_one(job):
        rel, cmd, _ = job
        r = subprocess.run(cmd, capture_output=True, text=True)
        return rel, r.returncode, r.stderr

    failed = False
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for n, (rel, rc, err) in enumerate(pool.map(compile_one, jobs), 1):
            if rc != 0:
                failed = True
                sys.stderr.write(f'\n{rel}:\n{err}')
            elif n % 40 == 0 or n == len(jobs):
                print(f'compiled : {n}/{len(jobs)}', flush=True)
    if failed:
        return 2

    stem = os.path.join(args.out, 'RosRobotControllerM4')
    elf, hexf, binf = stem + '.elf', stem + '.hex', stem + '.bin'
    link = ([cc] + LDFLAGS + ['-T' + os.path.join(fw, 'STM32F407VETX_FLASH.ld'),
                              f'-Wl,-Map={stem}.map', '-o', elf]
            + [obj for _, _, obj in jobs] + LIBS)      # link order = list order
    r = subprocess.run(link, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        return 2
    subprocess.run([objcopy, '-O', 'ihex', elf, hexf], check=True)
    subprocess.run([objcopy, '-O', 'binary', elf, binf], check=True)
    print(subprocess.run([size, elf], capture_output=True, text=True).stdout.rstrip())

    got = sha256(hexf)
    print(f'SHA-256  : {got}  {os.path.relpath(hexf, repo)}')
    if got == EXPECTED_HEX_SHA256:
        print('result   : IDENTICAL to the bench-validated image (pos-v8)')
        return 0
    print(f'expected : {EXPECTED_HEX_SHA256}')
    print('result   : DIFFERENT from the bench-validated image. This build has not '
          'been tested on hardware; check the compiler version first.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
