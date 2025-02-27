# PS1
This project is developed as part of a university assignment to create an automatic temperature regulation system using a arduino uno R4,a LM35 temperature sensor,a 9v bulb light,a 9v battery and a transistor.The requirements are:

A microcontroller-based driving system will be created that will perform the following functions:

-Heating the system up to a set temperature TSET in a time tincazlzire

-Maintaining this temperature for a period of time tmentinere

-Cooling the system gradually over a period of time tracire

The user interface:

-The system will have a 16x2 LCD on which the menu will be displayed, and during running the set temperature, the current temperature and the remaining time of the stage will be displayed current (tincalzire,tmentinere,tracire).

-The system menu will allow changing the following parameters: TSET,tincalzire,tmentinere,tracire,KP,KI,KD

-The parameters will be saved in the non-volatile memory. Rebooting the system will not affect saved parameters.

-Navigation through the equipment menu will be done with four buttons: "OK", "Cancel","+", "-".

Temperature regulation:

-Temperature control will be ensured by a PID type regulator

-The temperature sensor used will be an LM35 type or equivalent.

-The execution element of the system that will ensure the heating of the sensor will be a light bulb incandescent d.c. of power >= 5W which will be able to reach a minimum temperature 50°C

-A relay or transistor will be used to control the power on/off this bulb by the microcontroller.

