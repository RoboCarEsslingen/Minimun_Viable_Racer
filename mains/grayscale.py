### Imports
import sensor, pyb, math, time, mjpeg
from pyb import LED,Pin,Timer

### Parameters & constants
steering_direction = -1   # use this to revers the steering if your car goes in the wrong direction
steering_gain = 1.7  # calibration for your car's steering sensitivity
steering_center = 53  # set to your car servo's center point
kp = 0.8   # P term of the PID
ki = 0.0     # I term of the PID
kd = 0.4    # D term of the PID
thd = (240,255) #threshold for white lanes
MAG_THRESHOLD = 3 #magnitude of line in hough transform
old_error = 0
measured_angle = 0
set_angle = 90 # this is the desired steering angle (straight ahead)
p_term = 0
i_term = 0
d_term = 0
old_time = pyb.millis()
radians_degrees = 57.3 # constant to convert from radians to degrees
blue_led  = LED(3)
record_time=50000 #in ms
cruise_speed = 30 # we dont change it
roi_camera=(1,20,79,40) #bottom part of the field of view
GRAYSCALE_THRESHOLDS = [(240, 255)] # White Line.
GRAYSCALE_HIGH_LIGHT_THRESHOLDS = [(250, 255)]

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
    print('left angle: '+str(left)+' right angle: '+str(right))
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
imbin = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
imbin.replace(sensor.snapshot())
imbin.clear()
imbin.draw_rectangle(roi_camera[0],roi_camera[1],roi_camera[2],roi_camera[3],fill=True)

start = pyb.millis()
#while(pyb.elapsed_millis(start) < record_time):
deflection_angle=0
while(True):

    clock.tick()
    ######1. Perception
    #img = sensor.snapshot().histeq()
    #img = sensor.snapshot()
    img = sensor.snapshot().histeq()
    img.binary([thd])
    img.erode(1)
    #img.binary([thd],mask=imbin)
    #img.binary(GRAYSCALE_HIGH_LIGHT_THRESHOLDS, zero = True)
    #img.histeq()
    #img.binary(GRAYSCALE_THRESHOLDS)
    #img.erode(1, threshold = 5).dilate(1, threshold = 1)

    #img.erode(1)
    line = img.get_regression([(255,255)],roi=roi_camera, robust=True)
    img.draw_rectangle(roi_camera[0],roi_camera[1],roi_camera[2],roi_camera[3])
    ######2. Localization
    #deflection_angle = 0
    txt="no entry"
    if line and (line.magnitude() >= MAG_THRESHOLD):
        txt="entry"
        img.draw_line(line.line(), color=127)
        lane_center=40
        if abs(line.x1()-line.x2())<5:
            lane_center=int((line.x1()+line.x2())/2)
        elif abs(line.y1()-line.y2())<5:
            lane_center=100
        else:
            a=(line.y2()-line.y1())/(line.x2()-line.x1())
            b_1=line.y1()-a*line.x1()
            b_2=line.y2()-a*line.x2()
            b=(b_1+b_2)/2
            lane_center=(20-b)/a
        img.draw_cross(int(lane_center),20,255)
    ######3. Trajectory generation
        deflection_angle = -math.atan((lane_center-40)/40)
        deflection_angle = math.degrees(deflection_angle)
    #deflection_angle=50
    #print(txt+" angle: "+str(deflection_angle))
    #m.add_frame(img)
    ######4. Control
    now = pyb.millis()
    if  now > old_time + 0.02 :  # time has passed since last measurement; do the PID at 50hz
        blue_led.on()
        measured_angle = deflection_angle + 90
        steer_angle = update_pid()
        img.draw_string(5,5,str(round(steer_angle,2)),mono_space=False)
        old_time = now
        steer(steer_angle)
        blue_led.off()

pinADir0.value(0)
pinADir1.value(0)
pinBDir0.value(0)
pinBDir1.value(0)
#m.close(clock.fps())

