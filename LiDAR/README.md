1.) Goto https://www.slamtec.com/en/Support#rplidar-a-series


2.)Install Driver for the USB AdapterThe USB adapter converts UART to USB by using CP2102 chip. You need to installthe device driver for the chip. The driver can be found in the provided SDKpackage or downloaded from Silicon Labs' official website:http://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspxHere's the installation steps in Windows: after connecting the RPLIDAR with PC,please find the driver file "CP210x VCP Windows" and choose correct operatingsystem version accordingly: x86 for 32-bit OS and x64 for 64-bitOS


3.) You can now run tge Robo Studio Demo (https://www.manualslib.com/manual/1760808/Slamtec-Rplidar-A2-Series.html?page=7#manual) Follow the steps on here. 

4.) Other codes can be found here https://github.com/slamtec/rplidar_sdk#
-Follow the steps to extract the workspace then *in Visual Studio* build the code


Then run the examples:
 C:\Users\maksy\Desktop\LIDAR\rplidar_sdk-master\output\x64\Debug> .\ultra_simple --channel --serial \\.\COM3 115200

outputs:
theta: 333.41 Dist: 00000.00
   theta: 333.72 Dist: 00524.00
   theta: 334.31 Dist: 00522.00
   theta: 334.86 Dist: 00522.00
   theta: 335.39 Dist: 00520.00
   theta: 335.93 Dist: 00518.00
   theta: 336.47 Dist: 00514.00
   theta: 337.08 Dist: 00512.00
   theta: 337.60 Dist: 00512.00
   theta: 338.15 Dist: 00511.00
   theta: 338.69 Dist: 00509.00
   theta: 339.23 Dist: 00507.00
   theta: 339.76 Dist: 00505.00
   theta: 340.31 Dist: 00502.00
   theta: 340.91 Dist: 00501.00


 Or run 
 PS C:\Users\maksy\Desktop\LIDAR\rplidar_sdk-master\output\x64\Debug> .\frame_grabber --channel --serial \\.\COM3 115200
 ![image](https://github.com/user-attachments/assets/4e7d72c5-d23b-4da7-8e91-74708e19280f)
![image](https://github.com/user-attachments/assets/ecfc34c5-d140-4cc0-9dee-d43addcaf5e7)
