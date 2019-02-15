###This is a script to follow white or yellow lane
###Lane color must be selected in advance through by setting "Yellow" parameter
###if False then lane is White, else Yellow

### Imports
import sensor, pyb, math, time, mjpeg
from pyb import LED,Pin,Timer

### constants
steering_direction = -1   # use this to revers the steering if your car goes in the wrong direction
steering_gain = 1.7  # calibration for your car's steering sensitivity
steering_center = 53  # set to your car servo's center point
kp = 0.8   # P term of the PID
ki = 0.0     # I term of the PID
kd = 0.4    # D term of the PID
set_angle = 90 # this is the desired steering angle (straight ahead)
radians_degrees = 57.3 # constant to convert from radians to degrees
blue_led  = LED(3)

### parameters
Yellow=True
MAG_THRESHOLD = 3 #magnitude of line in hough transform
THD_WHITE=(240,255) #threshold for both yellow and white
THD_YELLOW=(180,255)
THD=THD_YELLOW if Yellow else THD_WHITE
old_error = 0
measured_angle = 0
p_term = 0
i_term = 0
d_term = 0
old_time = pyb.millis()
record_time=50000 #in ms
cruise_speed = 30 # we dont change it
width=79
height=30
x_0=1
y_0=10
x_middle=40
roi_camera=(x_0,y_0,width,height) #upper middle part of the field of view

####Helper functions
#keeps "value" within "min" and "max"
def constrain(value, min, max):
    if value < min :
        return min
    if value > max :
        return max
    else:
        return value
#produce steering angle for the car
def steer(angle):
    global steering_gain, cruise_speed, steering_center
    angle = int(round((angle+steering_center)*steering_gain))
    angle = constrain(angle, 0, 180)
    angle = 90 - angle
    angle = radians_degrees * math.tan(angle/radians_degrees) # take the tangent to create a non-linear response curver
    left = (90 - angle) * (cruise_speed/100)
    left = constrain (left, 0, 100)
    right = (90 + angle) * (cruise_speed/100)
    right = constrain (right, 0, 100)
    # Generate a 1KHz square wave on TIM4 with each channel
    ch1.pulse_width_percent(left)
    ch2.pulse_width_percent(right)
#updates car pid
def update_pid():
    global old_time, old_error, measured_angle, set_angle
    global p_term, i_term, d_term
    now = pyb.millis()
    dt = now - old_time
    error = set_angle - measured_angle
    de = error - old_error
    p_term = kp * error
    i_term += ki * error
    i_term = constrain(i_term, 0, 100)
    d_term = (de / dt) * kd
    old_error = error
    output = steering_direction * (p_term + i_term + d_term)
    output = constrain(output, -50, 50)
    return output

### Init: Motor pins
pinADir0 = pyb.Pin('P0', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE) #A is left wheel
pinADir1 = pyb.Pin('P1', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
pinBDir0 = pyb.Pin('P2', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE) #B is right wheel
pinBDir1 = pyb.Pin('P3', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)

pinADir0.value(1)
pinADir1.value(0)
pinBDir0.value(0)
pinBDir1.value(1)

tim = Timer(4, freq=1000) # Frequency in Hz
ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)
ch2 = tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width_percent=0)

### Init: Camera
clock = time.clock() # Tracks FPS.
start = pyb.millis()
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA) #80x60: recommended resolution for the processing we do
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 3000)

### Init: Recording file
fname1="gs_"+str(pyb.millis())+".mjpeg"
#m = mjpeg.Mjpeg(fname1)

### Init: Binary mask

start = pyb.millis()
deflection_angle=0
a=0.5*sensor.width()
lane_center=a
#while(pyb.elapsed_millis(start) < record_time):
while(True):
    clock.tick()
    ######1. Perception
    img = sensor.snapshot().histeq()
    img.binary([(180,255)])
    img.erode(1)
    line = img.get_regression([(255,255)],roi=roi_camera, robust=True)
    img.draw_rectangle(roi_camera[0],roi_camera[1],roi_camera[2],roi_camera[3])
    ######2. Localization
    if line and (line.magnitude() >= MAG_THRESHOLD):
        img.draw_line(line.line(), color=127)
        b=line.x1()
        if line.y2()< line.y1():b=line.x2()
        #if abs(line.x1()-line.x2())<5:
            #lane_center=int((line.x1()+line.x2())/2)
        lane_center=0.5*b+0.5*a
        a=b
        img.draw_cross(int(lane_center),y_0,127)
    ######3. Trajectory generation
        deflection_angle = -math.atan((lane_center-0.5*sensor.width())/(sensor.height()-y_0))
        deflection_angle = math.degrees(deflection_angle) #heading in degrees
    ######4. Control
    now = pyb.millis()
    if  now > old_time + 0.02 :  # time has passed since last measurement; do the PID at 50hz
        blue_led.on()
        measured_angle = deflection_angle + 90
        steer_angle = update_pid()
        txt=str(round(deflection_angle,2))
        img.draw_string(5,5,txt,scale=1,mono_space=False,color=127)
        old_time = now
        steer(steer_angle)
        blue_led.off()

pinADir0.value(0)
pinADir1.value(0)
pinBDir0.value(0)
pinBDir1.value(0)
#m.close(clock.fps())

