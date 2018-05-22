# -*- coding: utf-8 -*-
#testDemoStalkerStopLED.py

"""
Stalker function for Softbank Robotics Pepper robot.

Author Vako Varnkian and Mona Heggen in collaboration with OsloMet Robotics and Bachelor Thesis Group 1821 

20/5/2018

Project to make Pepper move and react to obstacles. 
"""

import time
import sys
import argparse
import qi
import math
import os, shutil 

import almath
from PIL import Image
from io import BytesIO
import StringIO
import vision_definitions 

nao_ip = "192.168.8.100"

class Explore(object):
    """Main class"""

    def __init__(self, app):
        super(Explore, self).__init__()
        
        app.start()
        session = app.session

        self.memory_service = session.service("ALMemory")
        self.motion_service = session.service("ALMotion")
        self.tts = session.service("ALTextToSpeech")
        self.autonomous = session.service("ALAutonomousLife")
        self.tracker_service = session.service("ALTracker")
        self.posture = session.service("ALRobotPosture")

        self.listen = session.service("ALListeningMovement")
        self.speak = session.service("ALSpeakingMovement")
        self.backround = session.service("ALBackgroundMovement")
        self.basic = session.service("ALBasicAwareness")

        self.leftBumper = self.memory_service.subscriber("LeftBumperPressed")
        self.rightBumper = self.memory_service.subscriber("RightBumperPressed")        
        self.backBumper = self.memory_service.subscriber("BackBumperPressed")

        self.leftBumper.signal.connect(self.onBumper)
        self.rightBumper.signal.connect(self.onBumper)
        self.backBumper.signal.connect(self.onBumper)

        # The robot gets in position to start the script.
        self.posture.goToPosture("StandInit", 2)

        #Disabeling some of Peppers basic features.
        self.listen.setEnabled(False)
        self.speak.setEnabled(False)
        self.backround.setEnabled(False)
        self.basic.setEnabled(True)
        self.basic.setTrackingMode("Head")
        self.basic.setStimulusDetectionEnabled("Sound", False)
        self.basic.setStimulusDetectionEnabled("TabletTouch", False)
        self.basic.setStimulusDetectionEnabled("Touch", False)

        self.autonomous.setSafeguardEnabled("RobotPushed", False)
        self.autonomous.setSafeguardEnabled("RobotMoved", False)
        self.autonomous.setSafeguardEnabled("RobotFell", False)

        #camera initializing: 
        resolution = vision_definitions.kQVGA  # 320 * 240
        colorSpace = vision_definitions.kRGBColorSpace
        #resolution =  2   # VGA
        #colorSpace = 11   # RGB
        self.video_service = session.service("ALVideoDevice")
        self.videoClient = self.video_service.subscribe("python_client", resolution, colorSpace, 5)

        self.led = session.service("ALLeds")

    
    def onBumper(self, value):
        """Bumper trigger reaction"""
        if value == 1:
            self.onStop()

    def motionRoll(self):
        print("robot is supposed to move | motionRoll")
        self.motion_service.moveInit()
        self.motion_service.move(0.175,0,0, _async=True)

    def onStop(self):
        self.turnLedOn("AllLeds", "red", 0.5)
        self.motion_service.moveInit()
        self.motion_service.move(0,0,0)

    def bodyTurn(self):
        print("turning body")
        self.turnLedOn("AllLeds", "cyan", 0.5)
        self.motion_service.moveInit()
        self.motion_service.moveTo(0,0,self.memory_service.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value"), _async=True)

    def onTurn(self, direction):
        #Turn logic
        self.turnLedOn("AllLeds", "magenta", 0.5)

        if direction == "right": #obstacle to peppers left, turning right
            self.motion_service.moveInit()
            self.motion_service.moveTo(0, 0, -(math.pi)/2, _async=True)
            print("onTurn Dir: Right")

        elif direction == "left":
            print("onTurn Dir: Left")
            self.motion_service.moveInit()
            self.motion_service.moveTo(0, 0, (math.pi)/2, _async=True)

        elif direction == "back": #sonar front has detected an obstacle
            if self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value") and self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value") == 1: # both IR sensors triggered, makes the robot back up
                print("Backing up")
                self.motion_service.moveInit()
                self.motion_service.moveTo(0, -1, 0, _async=True) # move back 1 meter
                self.motion_service.waitUntilMoveIsFinished()

    def turnLedOn(self, name, color, time):
        #colorName The name of the color (supported colors: “white”, “red”, “green”, “blue”, “yellow”, “magenta”, “cyan”).
        self.led.fadeRGB(name, color, time, _async=True)

    def takePicture(self):
        self.turnLedOn("AllLeds", "yellow", 0.5)
        print("take picture")

        # Capture image from Pepper
        naoImage = self.video_service.getImageRemote(self.videoClient)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]

        # Get byte array containing the image        
        array = naoImage[6]

        # Convert the byte array to a string and convert it 
        # to a PIL image using the width and height
        # used by Pepper when capturing it
        im = Image.frombytes("RGB", (imageWidth, imageHeight), str(array), "raw")

        # Save the image with the name YOLO expects 
        print("SAVE")
        im.save("./Output/picture/camImage.png", "PNG")   
        im.close()

        # Move the image to the location YOLO expects the image to be
        shutil.move("./Output/picture/camImage.png", "../YOLO/Input/pictureFromPepper/camImage.png")

    def cleanup(self):
        print("Interrupted by user, stopping HumanGreeter")

        # Go to Standig posture at half speed
        self.posture.goToPosture("StandInit", 0.5)
        self.scriptRunning = False
        self.onStop()
        # Release subscriptions
        self.memory_service.unsubscribe("LeftBumperPressed")
        self.memory_service.unsubscribe("RightBumperPressed")        
        self.memory_service.unsubscribe("BackBumperPressed")

        # Release the video client
        self.video_service.unsubscribe(self.videoClient)

        # Turn of the LED lights
        self.led.reset("ALLeds")
        sys.exit(0)

    def run(self):
        """Logic behind moving"""
        print("Script started")

        self.scriptRunning = True
        targetName = "Target"
        faceWidth = 0.10
        self.tracker_service.registerTarget(targetName, faceWidth)

        stop = "stop"        

        try:
            while self.scriptRunning == True:
                # Take a picture YOLO will try to detect hands in
                self.takePicture()
                inputFolder = "Input/Text/"
                if os.path.exists(inputFolder + stop):
                    # If the file 'stop' exist in the Input/Text folder, YOLO has detected hands
                    print("Stopping")
                    self.tts.say("Stopping!", _async=True)
                    self.onStop()
                else: # Implicit 'start' / keep on moving  
                    print("running")
                    if ( self.memory_service.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value") > 0.5 and self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value") == 0 and self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value")== 0):
                        self.tracker_service.track(targetName)
                        self.motionRoll()

                        self.Yaw = self.memory_service.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
                        if (self.Yaw > 0.45 or self.Yaw < -0.45): # 1.05 betyr 60 graders avvik fra midten av hode
                            self.bodyTurn()

                        print("Called by Sensor name: Front Sonar Value: " + str(self.memory_service.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value")))
                    
                    elif self.memory_service.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value") < 0.5:
                	    self.onStop()

                    elif (self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value") == 1):
                        print("InfraredLeft: " + str(self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value")))
                        self.onTurn("right")

                    elif (self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value") == 1):
                        print("InfraredRight: " + str(self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value")))
                        self.onTurn("left")
                    # When Both IR sensors trigger and the sonar sensor show a low value, Pepper turns around
                    elif (self.memory_service.getData("Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value") < 0.5) and (self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value") and self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value") == 1):
                    	print("InfraredRight: " + str(self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Right/Sensor/Value")))
                    	print("InfraredLeft: " + str(self.memory_service.getData("Device/SubDeviceList/Platform/InfraredSpot/Left/Sensor/Value")))
                    	self.onTurn("back")

            print("Exit script")
        
        except (KeyboardInterrupt, Exception) as e:
            print(e)
            sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default = nao_ip,
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["Explore", "--qi-url=" + connection_url])
    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    goex = Explore(app)
    goex.run()