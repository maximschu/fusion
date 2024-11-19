# IR Camera
The Waveshare Thermal USB Camera connects via USB to Windows, Android and Raspberry pi.

It has a thermal resolution of 80x62 pixels and a maximum frame rate of 25fps.

For more information see the wiki page here :
[Waveshare Wiki Page](https://www.waveshare.com/wiki/Thermal_Camera_HAT)


## Windows Guide
1. Plug in the IR Camera via USB
2. Download the following application: 
[Link](https://files.waveshare.com/wiki/Thermal-Camera-HAT/Thermal_USB_Camera_PC_Software.zip)

3. Run the application and click the connect button after selecting the correct serial port
GUIIMG
4. You should recieve an output similar to this
IMG
5. Clicking on a point will allow you to read off the temperature at a certain point
6. The bottom right hand corner has a record function, where you can capture a timeseries into a text file

NB: It is recommended to leave filtering on high. Additionally, if any issues arise, try pressing the physical reset button on the device


See the example file in this directory named ```SN152A01030FEF.txt``` which records the temperature values of every pixel for each frame of the image.

See scrappy example of thermal imaging at approximately 4m:
IMG