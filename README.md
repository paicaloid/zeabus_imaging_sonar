# zeabus_imaging_sonar
# Description
We use 64-bit Linux BlueView SDK libraries to implement Blueview Imaging Sonar M900-130 with ROS system for AUV name "ZEABUS" from Kasetsart University. You must have BlueView SDK before using zeabus_imaging_sonar package. 

Inside BlueView SDK folder, it has 6 folders
- colormaps
- data : store data in .son file
- doc : document for using SDK
- example : example code for c language
- include
- lib

# Set_up
1. After you extract blueView SDK, you will get bvtsdk folder and paste in some directory (For example, I paste in Home directory).
2. open .bashrc and add export BLUEVIEW_SDK_ROOT=/your_path_to_bvtsdk.
3. each file (.cpp or .py) in src directory, you need to change path in that code to your own path.  

# connect_sonar.cpp
connect Imaging sonar by using SDK. show and publish image by OpenCV and ROS

# open_son.cpp
open .son file in bvtsdk/data directory 

# record_son.cpp 
connect Imaging sonar by using SDK and record into .son file
