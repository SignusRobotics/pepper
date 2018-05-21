#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse
import almath
from PIL import Image
from io import BytesIO

nao_ip ="192.168.137.123"

class HumanGreeter(object):
    """
    A simple class to react to face detection events.
    """
    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(HumanGreeter, self).__init__()
        app.start()
        session = app.session
        # Get the service ALMemory.
        self.memory = session.service("ALMemory")
        # Connect the event callback.
        self.subscriber = self.memory.subscriber("FaceDetected")
        self.subscriber.signal.connect(self.on_human_tracked)
        
        # Get the services ALTextToSpeech and ALFaceDetection.
        self.tts = session.service("ALTextToSpeech")
        self.face_detection = session.service("ALFaceDetection")
        self.face_detection.subscribe("HumanGreeter")

        self.video_service = session.service("ALVideoDevice")
        resolution =  2   # VGA
        colorSpace = 11   # RGB

        self.videoClient = self.video_service.subscribe("python_client", resolution, colorSpace, 5)

        self.led = session.service("ALLeds")

        #Detected a face: 
        self.got_face = False
        
        #when Pepper is moving 
        self.moving = False

        # After 4 rotations, this is set to True
        self.finishedSearching = False
        
        # Get the service ALMotion.
        self.motion_service  = session.service("ALMotion")
        self.posture_service = session.service("ALRobotPosture")
        self.motion_service.setStiffnesses("Head", 1.0)
        # Set headpitch to zero position. 
        self.motion_service.setAngles("HeadPitch", 0, 0.02)

    def on_human_tracked(self, faces):
        """
        Callback for event FaceDetected.
        """
       
        if faces == []:  # empty value when the face disappears
            self.got_face = False
            print("No one")
        elif not self.got_face: # only speak the first time a face appears
            print("I saw a face!")
            self.got_face = True
            self.finishedSearching = True

    def moveHead(self, zrot):
        #self.tts.say("moving head")
        self.motion_service.setAngles("HeadYaw", zrot, 0.07)

    def moveBody(self, sensorAngles):
        if(not self.moving):
            self.moving = True 
            # Stops and then move head to the input angle (where the face was detected)
            # In practice this will either be no movement or back to where the head was
            # when the face was detected.
            self.moveHead(sensorAngles)

            # Tell that you're about to take a picture
            self.tts.say("Say cheese!")
            # Take picture
            self.takePicture()
            # Indicate that the picture has been taken
            self.tts.say("Thanks!")

            # Rotate body to the input angle (where the face was detected)
            self.motion_service.moveTo(0,0, sensorAngles)
            self.moveHead(0)

            # Move forward directy against where the face was detected
            self.motion_service.moveTo(0.5, 0, 0)
        else:
            print("why in moveBody?")
            #self.cleanup()
    
    def takePicture(self):
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
        im.save("camImage.png", "PNG")

    def turnLedOn(self, name, color, time):
        #colorName – The name of the color (supported colors: “white”, “red”, “green”, “blue”, “yellow”, “magenta”, “cyan”).
        # Fade in the LED light        
        self.led.fadeRGB(name, color, time)

    def cleanup(self):
        print("Interrupted by user, stopping HumanGreeter")
        # Go to Standig posture at half speed
        self.posture_service.goToPosture("StandInit", 0.5)
        # Release face detection subscription
        self.face_detection.unsubscribe("HumanGreeter")
        # Release the video client
        self.video_service.unsubscribe(self.videoClient)
        # Turn of the LED lights
        self.led.off("ALLeds")
        #stop
        sys.exit(0)

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        maxRotations = 4

        print("Starting HumanGreeter")
        # Go to Standig posture at half speed
        self.posture_service.goToPosture("StandInit", 0.5)
        # Get 90 degree angle in radians and turn it (look straight ahead)
        headAngleInput = 90*almath.TO_RAD
        self.moveHead(headAngleInput)

        try:
            while True:
                if self.finishedSearching:
                    # When Pepper finds a person. Turn on LEDs, rotate body 
                    # to where the head was when the person was detected

                    self.turnLedOn("AllLeds", "green", 0.5)
                    sensorAngles = self.motion_service.getAngles("Head", True)
                    #Flytter kroppen til denne angelen
                    self.moveBody(sensorAngles[0])

                # If Pepper has rotated 4 times it's time to stop searching
                elif maxRotations == 0:
                    # Turn on the LEDs with a red warning light    
                    self.turnLedOn("AllLeds", "red", 0.5)
                    print("I am finished rotating")
                    # Tell that search has been completed
                    self.tts.say("Finished Searching")
                    # Stand straight at half speed
                    self.posture_service.goToPosture("StandInit", 0.5)
                # Nothing found
                else: 
                    # Get the angle of the head
                    sensorAngles = self.motion_service.getAngles("Head", True)
                    # Indicate nothing found by turning on the LEDs in Cyan
                    self.turnLedOn("AllLeds", "cyan", 0.5)

                    # sensorAngles[0] is headyaw, around the Z-axis
                    print("Current: "),
                    print(round(sensorAngles[0],2))

                    desiredAngle = headAngleInput #* almath.TO_RAD
                    print("Desired: ",)  
                    print(round(desiredAngle, 2))

                    # If head is rotated 90 degrees
                    if round(sensorAngles[0],2) == round(desiredAngle, 2): 
                        # Indicate getting ready to turn by turning LEDs to Magenta
                        self.turnLedOn("AllLeds", "magenta", 0.5)
                        # Stand straight at half speed
                        self.posture_service.goToPosture("StandInit", 0.5)
                        # Turn to where head is looking
                        self.motion_service.moveTo(0.0, 0.0, sensorAngles[0])
                        # Turn head straight ahead
                        self.moveHead(headAngleInput)
                        # Decrement maxRotations to keep track of how many times it has rotated while searching                                                
                        maxRotations = maxRotations - 1
                print(maxRotations)
                time.sleep(1)
        except (KeyboardInterrupt, Exception) as e:
            print(e)
            self.cleanup()
            #pass

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
        app = qi.Application(["HumanGreeter", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    human_greeter = HumanGreeter(app)
    human_greeter.run()