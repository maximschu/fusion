# IR Camera
The Waveshare Thermal USB Camera connects via USB to Windows, Android and Raspberry pi.

It has a thermal resolution of 80x62 pixels and a maximum frame rate of 25fps.

For more information see the wiki page here :
[Waveshare Wiki Page](https://www.waveshare.com/wiki/Thermal_Camera_HAT)


## Windows Guide
1. Plug in the IR Camera via USB (USB-C port on Camera)
2. Download the following application: 
[Link](https://files.waveshare.com/wiki/Thermal-Camera-HAT/Thermal_USB_Camera_PC_Software.zip)

3. Run the application and click the connect button after selecting the correct serial port
![image](https://github.com/user-attachments/assets/34ba9096-cb8c-4fa3-a084-e1b561bf59c7)
4. You should recieve an output similar to this
![image](https://github.com/user-attachments/assets/cf697033-84c3-40bd-a2fa-517f5de5e54c)
5. Clicking on a point will allow you to read off the temperature at a certain point
6. The bottom right hand corner has a record function, where you can capture a timeseries into a text file

NB: It is recommended to leave filtering on high. Additionally, if any issues arise, try pressing the physical reset button on the device


See the example file in this directory named ```SN152A01030FEF.txt``` which records the temperature values of every pixel for each frame of the image.

See scrappy example of thermal imaging at approximately 4m:
![image](https://github.com/user-attachments/assets/e39c8ca2-2215-4db7-9981-7f68c9804210)


## Extracting Data Using USB
1. Download [pysenxor-master.zip](https://github.com/maximschu/fusion/blob/main/IR/pysenxor-master.zip) contained in this directory 
   (setup.py in this zip has been slightly altered for non raspberry pi usage - eliminates the need of SPI)

2. ```
   cd pysenxor-master
   ```

3. Type the following into terminal 
   ```
   pip install -e./
   ```
   This is mainly for the installation of the python library "pysenxor" for interacting with this specific thermal camera.
4. Contained inside the folder are also some example scripts.
   The most important ones are
    - **stream_usb.py** (live feed)
      
      <img width="300" alt="image" src="https://github.com/user-attachments/assets/15553fdf-e820-4b85-a000-3a23c0da79ac">

    - **single_capture_usb.py** (one single frame plotted using pyplot)
  
      <img width="350" alt="image" src="https://github.com/user-attachments/assets/07e87d55-a2c1-47f9-ae0c-5bdba5135956">

    - **senxor_mmx.py** (outputs three live frames: raw, filtered, segmented)
  
      <img width="713" alt="image" src="https://github.com/user-attachments/assets/b38a417b-2439-4106-a25b-15965120abd1">

    Anything with "_spi" can be ignored as they are for Raspberry Pi.
  
5. Temperature of each pixel is stored in an array (see csv example ```data_example.csv```)

6.  Here's pic with a colour/temperature scale for reference :)

   <img width="300" alt="image" src="https://github.com/user-attachments/assets/c9651b53-eb74-4fb5-9b81-d5a57cf1db9c">

   
