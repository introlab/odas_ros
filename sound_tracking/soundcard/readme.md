# Sound Card Rule Setup.

1. Unplug the 8SoundsUSB card
2. Open a terminal
3. Run this command: `udevadm monitor --kernel --subsystem-match=sound`
4. Plug the 8SoundsUSB card
5. The terminal displays something like this: 
```
KERNEL[942.482357] add      /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/sound/card1 (sound)
KERNEL[942.482476] add      /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/sound/card1/controlC1 (sound)
KERNEL[942.482724] add      /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/sound/card1/pcmC1D0p (sound)
KERNEL[942.482812] add      /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/sound/card1/pcmC1D0c (sound)
KERNEL[942.498162] change   /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/sound/card1 (sound)
```
6. Get the card name (in this case the name is /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/sound/card?)
7. Run this command: `sudo ./setup-sound-card-id.sh <card name>`
