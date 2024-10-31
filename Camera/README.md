# Intel Realsense D435 Viewer Set Up
1. https://github.com/IntelRealSense/librealsense/releases
2. Download Intel.RealSense.Viewer.exe
3. Use a USB-C 3.0 or higher cable

# Realsense SDK Viewer Examples
### Stereo Module and RGB Camera (2D VIEW) (Preset Settings: Default)
<img width="1127" alt="Screenshot 2024-10-06 143105" src="https://github.com/user-attachments/assets/f006c2e9-9674-41e7-bb64-f6af8b822379">

### Stereo Module and RGB Camera (2D VIEW) (Preset Settings: High Accuracy)
<img width="1128" alt="Screenshot 2024-10-06 143533" src="https://github.com/user-attachments/assets/1234ac6d-f798-4f7b-8f02-625cac50f586">

### Stereo Module and RGB Camera (3D VIEW)
<img width="1126" alt="Screenshot 2024-10-06 143225" src="https://github.com/user-attachments/assets/0e63e573-0c26-4714-a5f8-50b1867a418f">

### Depth (3D VIEW)
<img width="1128" alt="Screenshot 2024-10-06 143330" src="https://github.com/user-attachments/assets/cf76a35e-6324-484d-a95f-a7097e42c463">

# Explanation for Codes
| Filename        | Desciption           | Link |
| ------------- |:-------------:| -----:|
|yolo_depth.py | Uses YOLOv11n (trained off MS-COCO dataset containing 80 pre-trained classes) to detect object(s) and draws a bounding box around the identified objects. Depth of the object is determined by the centerpoint of the box. | [Link](https://github.com/maximschu/fusion/blob/main/Camera/yolo_depth.py)


