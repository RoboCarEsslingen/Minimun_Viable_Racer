#import sensor, image, time, pyb, mjpeg

#sensor.reset()
#sensor.set_pixformat(sensor.RGB565)
#sensor.set_framesize(sensor.QVGA)
#sensor.skip_frames(time = 2000)

#m = mjpeg.Mjpeg('openmv_video.mjpeg')

#clock = time.clock()
#start = pyb.millis()
#record_time=5000
#while(pyb.elapsed_millis(start) < record_time):
    #clock.tick()
    #m.add_frame(sensor.snapshot())
#print('closed record')
#m.close(clock.fps())

import sensor, gif

# Setup camera.
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames()

# Create the gif object.
g = gif.Gif("example.gif")

# Add frames.
for i in range(100):
    g.add_frame(sensor.snapshot())
    #print('saved')
# Finalize.
g.close()

