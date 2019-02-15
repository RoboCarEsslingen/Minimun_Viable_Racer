# Minimun_Viable_Racer
![Build Status](https://diyrobocars.com/wp-content/uploads/2017/10/IMG_20180421_223306-768x671.jpg)
This repository describes the code, installation and shares some link to get that MVR up and running !
In this project, we use opemv minimum viable racer configuration to successfully complete laps. Below is an expected outcome achieved using the provided code:
![follow lane]](http://www.youtube.com/watch?v=UrLFH2urBUM)

## Car mounting
https://diyrobocars.com/2017/10/01/a-minimum-viable-racer-for-openmv/

## Usage 
### Pre-requisites
1. Download & install openmvide
    https://openmv.io/pages/download
2. Connect rover to the ide
3. Prechecks
-Make sure the wheels work as expected by running motor-shield-powuer-driver_1.py
-Make sure the camera works properly by running hello world helloworld_1.py
4. Make sure to have some white tape and black paper.
### Vehicle setup
1. Before running the motor,make sure the camera is working properly by running `vehicle_setup/helloworld_1.py`
   You should expect to see a clear image similar to eye view.
   ![test camera](https://github.com/RoboCarEsslingen/Minimun_Viable_Racer/blob/master/images/testing_camera.png)
   If the image is blurry, need to adjust focal distance by tweaking the camera gently.
2. Make sure wheels spin properly by running `motor-shield-power-driver_1.py` Wheels should spin iteratively in both directions for 10 s.
   If not, then need to check: PINs, cable connections, battery, motor
### Run the vehicle
1. `git clone https://github.com/RoboCarEsslingen/Minimun_Viable_Racer`
2. In openmvide open `mains/lane_follow.py` 
3. Connect the car to the ide and click Tools > Save open script to Openmv cam

