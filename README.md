# HPACS
Hot Plate Automatic Control System
Videos: https://youtu.be/TlVIJFjHHOQ and https://youtu.be/JZ1UPQ5lsyg

For a bit more details look here: https://www.instructables.com/Hot-Plate-Automatic-Control-System-HPACS/

If you like then please

[!["Buy Me A Coffee"](https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png)](https://bmc.link/nic6911w)

## Purpose
This project aims at providing a simple intuitive way of understanding how to do Automatic PID tuning using a heater. What I have made is based on the Åström–Hägglund method for deriving parameters using bang-bang control to reveal system characteristics and subsequenctly chose parameters based on this knowledge. There is nothing secret to it and info can be found here: https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
And for chosing parameters you can read a bit here: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

To make it nice a Nextion 3.2" HMI interface is added for user input and displaying different variables real-time. BUT I also made a Serial terminal version of the library which makes the project way cheaper !

The real background story is that partly I promised my dad to make a temperature control for melting bees wax, partly I wanted to refresh my basic control theory and finally I wanted to look into auto tuning of PID controllers. On the side I also managed to use it for Sous Vide making nice steaks and Bearnaise sauce as part of the testing !

## Arduino based
The code is implemented on an Arduino Mega and uses two of the 16-bit timers for "PWM" and control loop. Furthermore, a Nextion 3.2" basic HMI panel is used on the HMI version of the code. I am not a software engineer so my coding may not be optimal or directly intuitive to others...

### Dependencies
I use a couple of libraries in the code: "DallasTemperature", "EEPROMex", "ITEADLIB_Arduino_Nextion-master" and the more commonly known "OneWire".

## Hardware/BOM
I use:
* A cheap WASCO Hot Plate 
* A cheap SS-relay
* A Dallas onewire temperature sensor 
* An Arduino Mega 
* (Optional) A Nextion 3.2" HMI interface/display

## License
For my sake this license will apply:
<a href="http://www.wtfpl.net/"><img
       src="http://www.wtfpl.net/wp-content/uploads/2012/12/wtfpl-badge-4.png"
       width="80" height="15" alt="WTFPL" /></a>
       
However, I have found the buttons for the HMI here: https://www.uihere.com/free-graphics/off-and-on-button-set-ai-eps-file-131170
Where it says "For Personal Use" under licensing - I.e. no commercialization of HPACS unless you change those buttons ;)
Also the disk icon and the hot surface warning I found somewhere at some point - but I can't seem to find them again...
