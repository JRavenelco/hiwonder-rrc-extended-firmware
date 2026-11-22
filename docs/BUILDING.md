# Building the firmware

There are three ways, for three purposes.

| Way | Use it to | Result |
|---|---|---|
| [A. Import the STM32CubeIDE project](#a-import-the-stm32cubeide-project) | read, change and debug the code | a new image, yours to validate |
| [B. Create the project from scratch](#b-create-the-project-from-scratch) | understand what the project consists of | the same project as A |
| [C. Reproducible build](#c-reproducible-build) | obtain the validated image from source | `pos-v8`, bit for bit |

Only C produces the image that was tested on the bench. An STM32CubeIDE build
uses the same sources and the same options, but the IDE's compiler is another
version and it links the objects in another order, so its bytes are different.
It has been compared with the validated image (the same program symbols and the
same interrupt vectors), not run on hardware. Treat an image you build in the
IDE as untested until you have tested it.

## A. Import the STM32CubeIDE project

Tested with STM32CubeIDE 2.3.0 on Linux, with the toolchain it bundles (GNU Tools
for STM32 14.3.rel1).

1. `File > Import... > General > Existing Projects into Workspace > Next`.
2. `Select root directory`: the `firmware` folder of this repository. Leave
   *Copy projects into workspace* unticked. `Finish`.
3. `Project > Build All` (the hammer).

The console ends with

```
   text    data     bss     dec     hex filename
  71540     176  102544  174260   2a8b4 RosRobotControllerM4-extended.elf
Build Finished. 0 errors, 17 warnings.
```

and `RosRobotControllerM4-extended.hex`, `.bin` and `.elf` are in `firmware/Debug`.
Sixteen of the warnings are in vendor code (unused variables, mostly). The last
one comes from the linker (`... has a LOAD segment with RWX permissions`) and is
a remark about the memory layout, not about the code.

*Debug* and *Release* differ only in the debug information: both use `-Os`, and
they produce the same HEX file.

From a terminal, without opening the IDE:

```bash
stm32cubeide --launcher.suppressErrors -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data /tmp/ws -import firmware -cleanBuild RosRobotControllerM4-extended/Release
```

### What the project contains

`firmware/.project` and `firmware/.cproject` started as the output of the IDE's
own wizard for an empty STM32F407VETx project. These are the settings that were
then changed, and why.

| Setting | Value | Why |
|---|---|---|
| MCU | STM32F407VETx, FPU `fpv4-sp-d16`, float ABI **hard** | set by the wizard; the firmware uses single-precision floats throughout |
| Source folders | `Core`, `Drivers`, `Hiwonder`, `Middlewares`, `Third_Party`, `USB_HOST` | everything in them is compiled: the 276 sources of the image and the two stub files described below |
| Symbols | `USE_HAL_DRIVER`, `STM32F407xx`, `LV_CONF_INCLUDE_SIMPLE`, `LV_LVGL_H_INCLUDE_SIMPLE` | select the HAL, the device header and LVGL's configuration header |
| Include paths | the 32 folders of `tools/include-dirs.txt` | the same search order as the validated build; `compat` goes last |
| Optimization | `-Os` | as the validated build; the control loop was tuned with this code |
| Other compiler flags | `-fno-strict-aliasing`, `-fcommon` | part of the validated configuration. The code was written for Arm Compiler and reads byte buffers through struct pointers; without `-fno-strict-aliasing` GCC generates different code. The image also links without `-fcommon`, but its bytes change |
| C library | newlib-nano, system calls `nosys.specs` | wizard defaults, and what the validated build uses |
| Linker script | `STM32F407VETX_FLASH.ld` | 512 KiB of flash at `0x08000000`, 128 KiB of RAM, 4 KiB stack, 2 KiB heap |
| Post-build | Intel HEX and binary | the HEX is what gets flashed |

**The two stub files.** `Core/Src/syscalls.c` and `Core/Src/sysmem.c` come from
the vendor's tree and are the files STM32CubeIDE generates for every project:
empty implementations of the system calls that the C library references
(`_write`, `_read`, `_close`, `_sbrk`...). The firmware does no file I/O, so they
only have to exist.

They are the one difference in sources between the two builds. The reproducible
build does not compile them: it takes the equivalent stubs from the toolchain
(`nosys.specs`), as the validated image does. The IDE build needs the files,
because without them the linker prints a note for each stub (`_close is not
implemented and will always fail`) and STM32CubeIDE reports the line before each
note as an error: a build that worked is shown as *Build Failed. 6 errors*.
Unticking *GNU Linker Error Parser* in the project properties does not help; the
linker tool brings that parser back.

## B. Create the project from scratch

This is how the project of this repository was built, and what the video shows.

1. **An empty project.** `File > STM32 Project Create/Import`, then
   `Create New STM32 Project > STM32CubeIDE Empty Project`, `Next`. In *Mcu Name
   filter* type `STM32F407VE`, select `STM32F407VETx`, `Next`. Name it
   `RosRobotControllerM4-extended`, language *C*, type *Executable*, `Finish`.
   The wizard creates `Src`, `Inc`, `Startup` and two linker scripts.
2. **Remove what the wizard generated.** Delete `Src`, `Inc`, `Startup` and both
   `.ld` files from the project: the firmware brings its own start-up code, its
   own linker script and its own copy of the stub files.
3. **Bring the sources in.** `File > Import... > General > File System`. *From
   directory*: the `firmware` folder of this repository. Tick the `firmware`
   folder to select everything, then untick `.project` and `.cproject` in the
   list on the right. *Into folder*: the new project. `Finish`.
4. **Include paths and symbols.** `Project > Properties > C/C++ General > Paths
   and Symbols > Import Settings...`, choose `firmware/cubeide-settings.xml` and
   the configuration, `Finish`. It adds the 32 include paths and the 4 symbols
   to that configuration (repeat it for *Release* if you build that one). The
   properties window closes by itself.
5. **Compiler and output options.** `Project > Properties > C/C++ Build >
   Settings`:
   - *MCU/MPU GCC Compiler > Optimization*: `Optimize for size (-Os)` and tick
     *Disable strict aliasing*;
   - *MCU/MPU GCC Compiler > Miscellaneous > Other flags*: add `-fcommon`;
   - *MCU/MPU Post build outputs*: tick *Convert to Intel Hex file*.
6. **Build.** `Project > Build All`.

The symbols and the include path that the wizard adds by itself (`DEBUG`,
`STM32`, `STM32F4`, `STM32F407VETx`, `../Inc`) can stay: the code does not use
them.

## C. Reproducible build

Needs Python 3 and the **Arm GNU Toolchain 15.2.Rel1** (`arm-none-eabi-gcc`) in
`PATH`. Nothing else: no IDE, no make.

```bash
python3 tools/build_reproducible.py
```

```
compiler : arm-none-eabi-gcc (Arm GNU Toolchain 15.2.Rel1 (Build arm-15.86)) 15.2.1 20251203
compiled : 276/276
   text    data     bss     dec     hex filename
  71720     176  102544  174440   2a968 .../build-reproducible/RosRobotControllerM4.elf
SHA-256  : 3a4d9e885ac408b11da56e7263b9e94c843f1b7bbbbf399f21cc93ab4a23f1e8  build-reproducible/RosRobotControllerM4.hex
result   : IDENTICAL to the bench-validated image (pos-v8)
```

The script compiles the files of `tools/link-order.txt` in that order, with the
include directories of `tools/include-dirs.txt` and the options written at its
top, links them with `firmware/STM32F407VETX_FLASH.ld` and compares the SHA-256
of the HEX with the validated one. The result is the same on x86-64 and on
aarch64 and does not depend on the directory the repository is in.

With another compiler version it still builds, prints `DIFFERENT` and exits with
status 1: that image is yours to test.

If you change the firmware, the script is also the quickest way to see that a
change you believed to be cosmetic really is: the SHA-256 must not move.

## The CubeMX file

`firmware/RosRobotControllerM4.ioc` is the vendor's STM32CubeMX configuration
(CubeMX 6.12.1, firmware package STM32Cube FW_F4 V1.28.0). It is kept as the
reference for pins, clocks and peripherals. **Do not regenerate code from it into
this tree**: a newer CubeMX and firmware package would replace the generated code
and the HAL that the image was validated with.
