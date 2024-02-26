# ch397 linux driver
## Description

USB2.0 to 100Mbps ethernet chip ch397 is fully compliant to the Communications Device Class (CDC) standard, it works with this vendor driver or standard CDC-ECM driver (CDC - Ethernet Networking Control Model). Linux operating systems supply a default CDC-ECM driver that can be used, the driver is cdc_ether.

The CDC-ECM driver has limited capabilities to control specific devices. This generic driver does not have any knowledge about specific device protocols. Because of this, device manufacturers can create an alternate, or custom driver that is capable of accessing the device specific function sets.

When use this vendor driver,  you needn't remove the cdc_ether driver cause this vendor driver supports automatic detection of chip mode and active switching in kernel version beyond 5.6.x, in lower versions, you need to actively uninstall the CDC-ECM driver. If you can modify the system kernel source code, you can add devices supported by the ch397 driver to the blacklist in cdc_ether.c file.

This driver supports USB2.0 to 100Mbps ethernet chip ch397.

1. Open "Terminal"
2. Switch to "driver" directory
3. Compile the driver using "make", you will see the module "ch397.ko" if successful
4. Type "sudo make load" or "sudo insmod ch397.ko" to load the driver dynamically
5. Type "sudo make unload" or "sudo rmmod ch397.ko" to unload the driver
6. Type "sudo make install" to make the driver work permanently
7. Type "sudo make uninstall" to remove the driver

Before the driver works, you should make sure that the usb device has been plugged in and is working properly, you can use shell command "lsusb" or "dmesg" to confirm that, USB VID is [1a86], you can view all IDs from the id table which defined in "ch397.c".

If the device works well, the driver will create a new net device, you can use shell command "ifconfig" to confirm the details.

## Note

If you need the driver under uboot, please refer to another repo.

Any question, you can send feedback to mail: tech@wch.cn