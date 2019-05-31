# Spyrodrive
Intelligent Synchronous Continuously Variable Transmission
ME3 Design Make and Test project, 10/2018 - 06/2019

Alexander MacLaren
James Baker
Josephine Wouda Kuipers
Navraj Lalli
Xin Yuan

Spyrodrive Final Build includes 5 files:
* finalBuild.ino contains the primary control flow, bearing the .ino extension for compilation by the Arduino IDE
* Spyrodrive.cpp contains the necessary helper functions for configuring peripherals and enabling data transfers
* Spyrodrive.h is the header accompanying Spyrodrive.cpp
* packetParser.cpp is a modified version of the packet parser by Kevin Townsend of Adafruit industries for managing packets sent by the Adafruit Bluefruit LE Connect app
* BluefruitConfig.h contains configuration settings for the Feather Bluefruit M0
