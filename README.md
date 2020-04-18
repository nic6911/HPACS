# HPACS
Hot Plate Automatic Control System

## Purpose
This project aims at providing a simple intuitive way of understanding how to do Automatic PID tuning using a heater. What I have made is based on the Åström–Hägglund method for deriving parameters using bang-bang control to reveal system characteristics and subsequenctly chose parameters based on this knowledge. There is nothing secret to it and info can be found here: https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
And for chosing parameters you can read a bit here: https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

Now, to make it really nice I will try to provide some easy to understand documentation on how the tuning and control functions at some point. I.e. explaining why and how this stuff works !
To make it nice a Nextion 3.2" HMI interface is added for user input and displaying different variables real-time.

The real background story is that partly I promised my dad to make a temperature control for melting bees wax, partly I wanted to refresh my basic control theory and finally I wanted to look into auto tuning of PID controllers. On the side I also managed to use it for Sous Vide making nice steaks and Bearnaise sauce as part of the testing !

## Arduino based
The code is implemented on an Arduino Mega and uses two of the 16-bit timers for "PWM" and control loop. Furhtermore a Nextion 3.2" basic HMI panel is used. I am not a software engineer so my coding may ot be optimal or directly intuitive to others...

## Hardware
I use a cheap WASCO Hot Plate, a cheap SS-relay, a Dallas onewire temperature sensor an Arduino Mega and a Nextion 3.2" HMI interface/display.

## License
<a href="http://www.wtfpl.net/"><img
       src="http://www.wtfpl.net/wp-content/uploads/2012/12/wtfpl-badge-4.png"
       width="80" height="15" alt="WTFPL" /></a>
