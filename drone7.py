from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
from pymavlink import mavutil
import cv2
import numpy as np
from threading import Thread
import imutils
import math
import threading
import serial

#import RPi.GPIO as GPIO

"""
class sensor:
    def get_distance(self):
        GPIO.setmode(GPIO.BCM)
        GPIO_TRIGGER = 18
        GPIO_ECHO = 24
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN)
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300) / 2
        return distance
"""




"""
class mekanizma:    
    def pump(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.OUT)
        GPIO.setup(24, GPIO.OUT)
        GPIO.output(23, True)
        GPIO.output(24, True)
        time.sleep(20)
        GPIO.cleanup()       
    def openn(self):      
        servoPIN = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPIN, GPIO.OUT)
        p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
        p.start(2.5) # Initialization
        p.ChangeDutyCycle(3)
        time.sleep(4)
        p.stop()
        GPIO.cleanup()

"""





class drone:

    def __init__(self,connection_string):
        self.connection_string = connection_string
        self.vehicle = connect(self.connection_string, wait_ready=True, baud=57600)
        self.image_process = image_processer()
        #self.mek = mekanizma()
        self.t_land = threading.Thread(target=self.land)
        self.t_data_read = threading.Thread(target=self.data_read)
        self.stop_land = False
        self.stop_data_read = False
        #self._setup_listeners()
        self.att_roll_deg=0
        #self.ser = serial.Serial('/dev/ttyUSB1',57600,timeout=0.5)
        #self.sensor=sensor()



    def data_read(self):
        self.data = ""

        speed = "0"
        alt = "0"
        pitch="0"
        roll="0"
        heading="0"
        mode="0"
        x = "0"
        lat = "0"
        long="0"
        while True:
            self.data = ""
            x = str(0)
            speed = str(round(self.vehicle.airspeed,2))
            alt = str(round(self.vehicle.location.global_relative_frame.alt,2))
            pitch = str(round(math.degrees(self.vehicle.attitude.pitch),2))
            roll= str(round(math.degrees(self.vehicle.attitude.roll),2))
            heading = str(self.vehicle.heading)
            mode = str(self.vehicle.mode.name)
            lat = str(self.vehicle.location.global_relative_frame.lat)
            long = str(self.vehicle.location.global_relative_frame.lon)
            self.data =  x+","+speed+","+alt+","+pitch+","+roll+","+heading+","+mode+","+lat+","+long         
            print(self.data)
            #self.ser.write(self.data+'\n'.encode('utf-8'))        
            time.sleep(0.1)
            if self.stop_data_read==True:
                break
    
    
    def channels(self,channel):
        print(" ch8 : %s " % self.vehicle.channels[channel])
        return self.vehicle.channels[channel]


    def arm_disarm(self,value):
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        if(value=="1"):
            self.vehicle.armed = True
            print("armed")
        else:
            self.vehicle.armed = False
            print("Disarmed")


    def mode(self,mode):
        self.vehicle.mode = str(mode)
        print("mode:", mode)


    def takeoff(self,h):
        print("Taking off!")
        self.vehicle.simple_takeoff(h)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            print("alt2:",self.vehicle.location.global_relative_frame.alt* 1e-3)
            if self.vehicle.location.global_relative_frame.alt >= h * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)



    def readmission(self,file):
        list = []
        dosya = open(file, 'r')
        for satir in dosya:
            try:
                Line = satir.split(",")
                x = float(Line[0])
                y = float(Line[1])
                h = int(Line[2].strip())
                # h = int(Line[3].strip())
                print("x :", x, "y :", y)
                cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                              0,
                              0, 0,
                              0, 0,
                              0, x, y, h)
                list.append(cmd)
            except:
                print("erol")
        dosya.close()
        print(list)
        return list



    def upload_mission(self,file):

        missionlist = self.readmission(file)

        cmds = self.vehicle.commands
        cmds.clear()
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                    0, 0,
                    0,
                    0, 0, 0, 5))
        for command in missionlist:
            cmds.add(command)
        print(' Upload mission')
        self.vehicle.commands.upload()
        # Add new mission to vehicle

    def ach_image(self,wp_no,count,color):
        i=0
        while True:
            nextwaypoint = self.vehicle.commands.next
            msg1, cx, cy = self.image_process.image2(count,color)
            print("msg1:", msg1)
            if(i==0):
                if(msg1!=7):
                    self.waypoint_save()
                    print("waypoint")
                    i=1
            if (nextwaypoint == wp_no):
                break

                

    def ach_route(self, wp_no):
        while True:
            nextwaypoint = self.vehicle.commands.next
            print("waypoint_no :", nextwaypoint, "wp_no :", wp_no)
            if nextwaypoint == wp_no:
                break

    def waypoint_save(self):
        i = 0
        print(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon)
        Lat = self.vehicle.location.global_frame.lat
        Lon = self.vehicle.location.global_frame.lon
        dosya2 = open("rota3.txt", 'a+')
        satir1 = str(Lat)
        satir2 = str(Lon)
        dosya2.write("\n" + satir1 + "," + satir2 + "," + str(10))
        dosya2.write("\n" + satir1 + "," + satir2 + "," + str(10))
        dosya2.close()

    def move(self, hiz_x, hiz_y, hiz_z, sure):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
            0b0000111111000111,  # type_mask
            0, 0, 0,  # x, y, z positions (not used)
            hiz_x, hiz_y, hiz_z,  # m/s
            0, 0, 0,  # x, y, z acceleration
            0, 0)
        for x in range(0, sure):
            self.vehicle.send_mavlink(msg)
            print(x)
            time.sleep(1)
    
    def land(self):
        while True:
            self.move(0, 0, 0.3,1)
            print("move")
            if (self.stop_land==True):
                break
    
    def land_control(self,alt):
        self.t_land.start()
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(current_alt)
            if(current_alt<alt):
                self.stop_land=True
                break

    
    
    def land_to_alt(self, alt):

        alt = float(alt)
        
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print("current_alt:",current_alt,"hiz:0.5")
            self.move(0, 0, 0.5, 1)
            time.sleep(1)
            if(current_alt<7):
                break
        
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print("current_alt:", current_alt,"hiz:0.2")
            self.move(0, 0, 0.3, 1)
            time.sleep(1)
            if (current_alt < alt):
                break
        





    def target_aim(self,msg1,cx,cy):

        f=0

        if(msg1==1):
            f=1
        if(msg1==0):
            print("obje yok")
            self.move(-0.4, 0, 0, 1)  

        if (msg1 == 2):
            self.move(0.4, -0.4, 0, 1)  # ileri ve sol

        if (msg1 == 3):
            self.move(0.4, 0, 0, 1)  # ileri

        if (msg1 == 4):
            self.move(0.4, 0.4, 0, 1)  # ileri ve sag

        if (msg1 == 5):
            self.move(0, -0.4, 0, 1)  # sol

        if (msg1 == 6):
            self.move(0, 0.4, 0, 1)  # sag

        if (msg1 == 7):
            self.move(-0.4, -0.4, 0, 1)  # geri ve sol

        if (msg1 == 8):
            self.move(-0.4, 0, 0, 1)  # geri

        if (msg1 == 9):
            self.move(-0.4, 0.4, 0, 1)  # geri ve sag

        return f

    def aim_moving_2(self, cx, cy):
        F=0
        if(cy<213):
            if(cx<284):
                print("7")
                self.move(-0.2, -0.2, 0, 1)# geri ve sol
            if(284<cx<355):
                print("5")
                self.move(0, -0.2, 0, 1)# sol
            if(cx>355):
                print("2")
                self.move(0.2, -0.2, 0, 1)# ileri ve sol
        if (213 < cy < 266):
            if (cx < 284):
                print("8")
                self.move(-0.2, 0, 0, 1)# geri
            if (284 < cx < 355):
                print("Basarili")
                F=1               
            if (cx > 355):
                print("3")
                self.move(0.2, 0, 0, 1)# ileri
        if (cy > 266):
            if (cx < 284):
                print("9")
                self.move(-0.2, 0.2, 0, 1)# geri ve sag
            if (284 < cx < 355):
                print("6")
                self.move(0, 0.2, 0, 1) # sag
            if (cx > 355):
                print("4")
                self.move(0.3, 0.2, 0, 1)# ileri ve sag
       
        return F

    

    def aim(self,count,color):
        while True:
            msg1,cx,cy = self.image_process.image(count,color)
            f  = self.target_aim(msg1,cx,cy)
            if(f==1):
                break
        while True:
            msg1,cx, cy = self.image_process.image(count,color)
            f = self.aim_moving_2(cx, cy)
            if (f == 1):
                print("cisim ortada")
                break



class image_processer():

        class WebcamVideoStream:
            def __init__(self, src=0, name="WebcamVideoStream"):
                # initialize the video camera stream and read the first frame
                # from the stream
                self.stream = cv2.VideoCapture(src)
                (self.grabbed, self.frame) = self.stream.read()

                # initialize the thread name
                self.name = name

                # initialize the variable used to indicate if the thread should
                # be stopped
                self.stopped = False

            def start(self):
                # start the thread to read frames from the video stream
                t = Thread(target=self.update, name=self.name, args=())
                t.daemon = True
                t.start()
                return self

            def update(self):
                # keep looping infinitely until the thread is stopped
                while True:
                    # if the thread indicator variable is set, stop the thread
                    if self.stopped:
                        return

                    # otherwise, read the next frame from the stream
                    (self.grabbed, self.frame) = self.stream.read()

            def read(self):
                # return the frame most recently read
                return self.frame

            def stop(self):
                # indicate that the thread should be stopped
                self.stopped = True

        def __init__(self):
            self.stream = self.WebcamVideoStream(src=0).start()


        def image(self,count,color):

            first_time = time.time()

            cx=0
            cy=0
            msg1=0

            frame = self.stream.read()
            frame = cv2.resize(frame, (640, 480))


            red_lower = np.array([0, 142, 72])
            red_upper = np.array([9, 255, 255])

            lower_blue = np.array([0, 22, 192])
            upper_blue = np.array([255, 255, 255])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            if (color == "red"):
                mask = cv2.inRange(hsv, red_lower, red_upper)
                print("red scaning..")
            if (color == "blue"):
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
                print("blue scaning..")


            kernal = np.ones((5, 5), "uint8")

            hsv_mask = cv2.GaussianBlur(mask, (11, 11), 0)

            hsv_mask = cv2.erode(hsv_mask, None, iterations=2)
            hsv_mask = cv2.dilate(hsv_mask, None, iterations=2)
            
            x1 = 0
            x2 = 640
            y1 = 0
            y2 = 480
            tt = 1
            x = 0
            y = 0
            cx = 0
            cy = 0
            msg1 = 0
            msg2 = 0
            while (y1 < y2):
                while (x1 < x2):
                    if (hsv_mask[y1, x1] > 5):
                        cx = cx + x1
                        cy = cy + y1
                        tt = tt + 1

                    x1 += count
                x1 = 0
                y1 += count

            cx = int(cx / tt)
            cy = int(cy / tt)            
            
            if (cx < 213):
                if (cy < 133):
                    msg1 = 7
                if (160 < cy < 320):
                    msg1 = 8
                if (cy > 320):
                    msg1 = 9

            if (213 < cx < 426):
                if (cy < 160):
                    msg1 = 5
                if (160 < cy < 320):
                    msg1 = 1
                if (cy > 320):
                    msg1 = 6

            if (cx > 426):
                if (cy < 160):
                    msg1 = 2
                if (160 < cy < 320):
                    msg1 = 3
                if (cy > 320):
                    msg1 = 4


            second_time = time.time()

            print("msg1:",msg1,"cx:",cx,"cy:",cy,"FPS:",1/(second_time-first_time))

            cv2.line(frame, (213, 0), (213, 480), (0, 0, 255), 2)
            cv2.line(frame, (0, 160), (640, 160), (0, 0, 255), 2)
            cv2.line(frame, (0, 320), (640, 320), (0, 0, 255), 2)
            cv2.line(frame, (426, 0), (426, 480), (0, 0, 255), 2)
            cv2.line(frame, (213,213), (426, 213), (0, 0, 255), 2)
            cv2.line(frame, (213, 266), (426, 266), (0, 0, 255), 2)
            cv2.line(frame, (284, 213), (284, 266), (0, 0, 255), 2)
            cv2.line(frame, (355, 213), (355, 266), (0, 0, 255), 2)
            
            
            cv2.circle(frame, (cx, cy), 10, (255, 0, 0), 2)

            cv2.imshow("final_frame",frame)

            k = cv2.waitKey(1) & 0xFF

            return msg1,cx,cy




        def image2(self, count, color):

            first_time = time.time()

            cx = 0
            cy = 0
            msg1 = 0

            frame = self.stream.read()
            frame = cv2.resize(frame, (400, 400))

            red_lower = np.array([0, 142, 72])
            red_upper = np.array([9, 255, 255])

            lower_blue = np.array([0, 22, 192])
            upper_blue = np.array([255, 255, 255])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            if (color == "red"):
                mask = cv2.inRange(hsv, red_lower, red_upper)
                print("red scaning..")
            if (color == "blue"):
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
                print("blue scaning..")

            kernal = np.ones((5, 5), "uint8")

            hsv_mask = cv2.GaussianBlur(mask, (11, 11), 0)

            hsv_mask = cv2.erode(hsv_mask, None, iterations=2)
            hsv_mask = cv2.dilate(hsv_mask, None, iterations=2)

            x1 = 0
            x2 = 400
            y1 = 0
            y2 = 400
            tt = 1
            x = 0
            y = 0
            cx = 0
            cy = 0
            msg1 = 0
            msg2 = 0
            while (y1 < y2):
                while (x1 < x2):
                    if (hsv_mask[y1, x1] > 5):
                        cx = cx + x1
                        cy = cy + y1
                        tt = tt + 1

                    x1 += count
                x1 = 0
                y1 += count

            cx = int(cx / tt)
            cy = int(cy / tt)




            if (cx < 133):
                if (cy < 133):
                    msg1 = 7
                if (133 < cy < 266):
                    msg1 = 8
                if (cy > 266):
                    msg1 = 9

            if (133 < cx < 266):
                if (cy < 133):
                    msg1 = 5
                if (133 < cy < 266):
                    msg1 = 1
                if (cy > 266):
                    msg1 = 6

            if (cx > 266):
                if (cy < 133):
                    msg1 = 2
                if (133 < cy < 266):
                    msg1 = 3
                if (cy > 266):
                    msg1 = 4



            second_time = time.time()

            print("msg1:", msg1, "cx:", cx, "cy:", cy, "FPS:", 1 / (second_time - first_time))


            cv2.line(frame, (133, 0), (133, 400), (0, 0, 255), 2)
            cv2.line(frame, (266, 0), (266, 400), (0, 0, 255), 2)
            cv2.line(frame, (0, 133), (400, 133), (0, 0, 255), 2)
            cv2.line(frame, (0, 266), (400, 266), (0, 0, 255), 2)


            cv2.circle(frame, (cx, cy), 10, (255, 0, 0), 2)

            cv2.imshow("final_frame", frame)

            k = cv2.waitKey(1) & 0xFF

            return msg1, cx, cy



































