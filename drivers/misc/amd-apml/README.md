.. SPDX-License-Identifier: GPL-2.0
# amd apml modules (apml_sbtsi, apml_sbrmi)

amd-apml: APML interface drivers for BMC

EPYC processors from AMD provide APML interface for
BMC users to monitor and configure the system parameters
via the Advanced Platform Management List (APML) interface
defined in EPYC processor PPR.

This chapter defines custom protocols over i2c/i3c bus
  - Mailbox
  - CPUID [RO]
  - MCA MSR {RO]
  - RMI/TSI register [RW]

module_i2c_i3c based sbrmi and sbtsi modules, which
are probed as i2c or i3c client devices, depending on the
platforms DTS.

https://developer.amd.com/resources/epyc-resources/epyc-specifications

APMl library provides C API fo the user space application on top of this
module.


Disclaimer
===========

The amd apml modules are supported only on AMD Family 19h (including
third-generation AMD EPYC processors (codenamed "Milan")) or later
CPUs. Using the amd apml modules on earlier CPUs could produce unexpected
results, and may cause the processor to operate outside of your motherboard
or system specifications. Correspondingly, defaults to only executing on
AMD Family 19h Model (0h ~ 1Fh & 30h ~ 3Fh) server line of processors.

Interface
---------

Both apml_sbtsi and apml_sbrmi modules register a misc_device
to provide ioctl interface to user space, allowing them
to run these custom protocols.

apml_sbtsi module registers hwmon sensors for monitoring
current temperature, managing max and min thresholds.

apml_sbrmi module registers hwmon sensors for monitoring
power_cap_max, current power consumption and managing
power_cap.


Build and Install
-----------------

Kernel development packages for the running kernel need to be installed
prior to building the amd apml modules. A Makefile is provided which should
work with most kernel source trees.

To cross compile for arm based BMC

export CC=arm-openbmc-linux-gnueabi-gcc # Or similar
export ARCH=arm
KDIR=<Path to prebuilt BMC kernel>

To build the kernel module:

#> make

To install the kernel module:

#> sudo make modules_install

To clean the kernel module build directory:

#> make clean


Note: There is a fix required in the upstream linux kerenl header to handle the i3c_dev.
the patch is kept in patches/ folder of this repo.

Loading
-------

If the apml modules were installed you should use the modprobe command to
load the module.

#> sudo modprobe apml_sbrmi apml_sbtsi
