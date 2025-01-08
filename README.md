# OpenBaitCaster
Open source Bait Casting Reel controller

## The Goals

1. Firmware for Arduino  
The firmware is designed for the smallest 32bits Arduino or [Seeeduino Xiao BLE](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html). The mbedOS working behind these arduino family hes made the multi threading easy and at the same time reliable. 

2. Realtime direct line tension control  
The realtime control is designed for the control of reel. It directly controls the line tension with the quick change of the pwm duty. This results the brake intensity to adapt automatically to any lure and wind condition without the selection of such as rotary switches.

3. Minimal Circuit  
The circuit is comprised with only discrete devices, no need of the special devices such as a Hole-IC to detect spool rotation.

4. Design rules of principal parts  
The principal parts are a coil and a magnet rotor. We reveal the design rule for these parts.

5. Monitoring and Setting tools  
Due to the usage of casting reel, the wireless communication is needed for testing.

