#!/bin/sh

if [ -x /usr/bin/sudo -a $UID -ne 0 ]; then
   exec /usr/bin/sudo $0 $*
fi

sysctl -w debug.iokit=0x200000

echo "This will install the Envy24 driver on your system."

if [ -d /System/Library/Extensions/Envy24PCIAudioDriver.kext ]; then
	kextunload /System/Library/Extensions/Envy24PCIAudioDriver.kext > /dev/null
	kextunload /System/Library/Extensions/Envy24PCIAudioDriver.kext > /dev/null
	rm -R /System/Library/Extensions/Envy24PCIAudioDriver.kext
fi

cp -R Envy24PCIAudioDriver.kext /System/Library/Extensions/

find /System/Library/Extensions/Envy24PCIAudioDriver.kext -type d -exec /bin/chmod 0755 {} \;
find /System/Library/Extensions/Envy24PCIAudioDriver.kext -type f -exec /bin/chmod 0744 {} \;
chown -R root:wheel /System/Library/Extensions/Envy24PCIAudioDriver.kext

kextload /System/Library/Extensions/Envy24PCIAudioDriver.kext

echo "Installation finished"
