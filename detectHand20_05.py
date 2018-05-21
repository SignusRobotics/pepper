from darkflow.net.build import TFNet
import cv2

from PIL import Image, ImageDraw, ImageFont
import numpy as np

import os, shutil
from time import sleep

# Default state to allow Pepper to move
state = "start"
# Folder Pepper looks in to know if it should start or stops
stateFolder = "../PepperBilde/Input/Text/"

# Helper function for writing a file to disk with the current status
# If the previous state has an existing file, it'll rename it to the
# new one.
# If no previous state is passed in it'll use 'first', guaranteeing
# that a new file with the new state as file name will be written to disks
def WriteResultToDisk(status, prevStatus = "first"):
    if os.path.exists(stateFolder + prevStatus): 
        # 'move' file to new file with the name of status:
        shutil.move(stateFolder + prevStatus, stateFolder + status)
    else:   
        # create a file with file name equal to status
        hs = open(stateFolder + status, "w+")
        hs.close()

# If state from last run is stop, set it to 'start'
WriteResultToDisk("start","stop")

# Configure the YOLO model, its weights and minimal threshold. 
options = {"model": "cfg/yolov2-tiny-obj.cfg", "load": "bin/yolov2-tiny-obj_30700.weights", "threshold": 0.2}

# Init YOLO with with the options
tfnet = TFNet(options)
# Counter used as a killswitch while testing. 
counter = 0

# Folder where images to detect objects are saved
YOLOinput =  "Input/pictureFromPepper/"
# Archive folder
archive = "Arkiv/"

# Only using the counter when testing, would use True
# or a variable that listens to exeternal commands to know
# when to quit.
while counter < 1000:
    # Image to check will always have the name 'camImage.png'
    # 'camImage.png' will always be the last image captured on Pepper
    filename = "camImage.png" 
    # If a new image from Pepper has been captured:
    if os.path.exists(YOLOinput + filename): 
        # Rename the image. This allows Pepper to capture and save new images
        # without interrupting the object detection.
        filename2 = "temp.png"
        os.rename(YOLOinput + filename, YOLOinput + filename2)

        # Load and convert the image to RGB for further processing.
        imgcv = Image.open(YOLOinput+filename2).convert('RGB')
        
        # Extract 'height' and 'width' in pixels from the image
        width, height = imgcv.size

        # Convert to Open CV format from RGB to BGR.
        curr_imgcv2 = cv2.cvtColor(np.array(imgcv), cv2.COLOR_RGB2BGR)

        # Predict objects in the image using the trained model loaded
        result = tfnet.return_predict(curr_imgcv2)

        # Hook up ImageDraw to the original image. This'll allow us to draw
        # bounding boxes.
        draw = ImageDraw.Draw(imgcv)

        # Loop through all objects detected by YOLO
        for det in result:
            # Label is what the predictor belives the object is
            print(det['label']), 
            # Confidence is how sure the predictor is that the object is what it says it is.
            print(det['confidence'])
            
            if det['confidence'] > 0.4:
                # If the predictor is above the threshold, just assume it's a hand 
                # and drop blob/keypoints detection. 
                # Go straight to sending the stop signal.            
                output = "stop" 
                print("State is stop")
                print(output)
                WriteResultToDisk(output, state)
                state = output

                # Hand detected, break out of loop to avoid starting if next object detected has to low confidence
                break 
            elif det['confidence'] > 0.20:
                # This could be a hand. YOLO is not as certain, but by checking for blobs with
                # approx. of skin color and more than minimum of detected keypoints

                # Draw bounding around the object detected
                boundingBox = draw.rectangle([det['topleft']['x']-20, det['topleft']['y']-20, 
                                det['bottomright']['x']+20, det['bottomright']['y']+20],
                                outline=(0, 0, 255))
                
                # Print all the data the detector has for the object (debug)
                print(det)

                # We expand the bounding box to ensure that all fingers are 
                # included in the image before cropping and analyzing it.
                if  det['topleft']['y'] <= 20 or det['topleft']['x'] <= 20 or det['topleft']['y'] + 20 > height or det['topleft']['x'] + 20:
                    # Expanding the bounding box would go outside one or more of the image edges. 
                    # Keep the bounding box YOLO has decided on.
                    # Crop it.
                    rect_img2 = curr_imgcv2[
                        det['topleft']['y'] : det['bottomright']['y'],  
                        det['topleft']['x'] : det['bottomright']['x']] 
                else:    
                    # Expand by 20 pixels in all directions.
                    # Crop it.
                    rect_img2 = curr_imgcv2[
                        det['topleft']['y']-20 : det['bottomright']['y']+20,  
                        det['topleft']['x']-20 : det['bottomright']['x']+20]
                
                # Save the cropped image (mostly for being able to start from this point when testing).
                cv2.imwrite("./Output/testHand.png", rect_img2)

                # Load the cropped image using OpenCV
                imgblob = cv2.imread("./Output/testHand.png") 

                # Convert BGR to HSV
                hsv = cv2.cvtColor(imgblob, cv2.COLOR_BGR2HSV)

                # Define range of light skin color in HSV: 
                lower = np.array([0,48,80], dtype = "uint8")
                upper = np.array([20,255,255], dtype = "uint8")
                
                # Create a mask using the lower and upper color boundaries.
                mask = cv2.inRange(hsv, lower, upper)
                # Bitwise AND mask results in an image where pixels within the range of the mask are
                # white and all others black.
                res = cv2.bitwise_and(imgblob, imgblob, mask = mask)
                # Save the masked image (mostly for being able to start from this point when testing)-
                cv2.imwrite("./Output/blob2.png", res)
                
                # Load the masked image using OpenCV
                im = cv2.imread("./Output/blob2.png", cv2.IMREAD_GRAYSCALE)
        
                # Get the default SimpleBlobDetector parameters.
                params = cv2.SimpleBlobDetector_Params()
                
                # Change thresholds 
                params.minThreshold = 10
                params.maxThreshold = 200

                # Filter by 
                params.filterByColor = True
                params.blobColor = 255

                # Filter by Area. 
                params.filterByArea = True
                params.minArea = 0

                # Filter by Convexity
                params.filterByConvexity = True
                params.minConvexity = 0.50

                # Filter by Inertia
                params.filterByInertia = True
                params.minInertiaRatio = 0.01

                # Create a detector with the parameters
                detector = cv2.SimpleBlobDetector_create(params)

                # Detect keypoints in blob.
                keypoints = detector.detect(im)

                # Draw the keypoints to a image.
                im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                # Save the image with the keypoints written to it.
                cv2.imwrite("./Output/key.png", im_with_keypoints)

                # Print the size of the keypoints array 
                print ("Keypoints length %d" %len(keypoints))

                # If the size of color in total is large enough we categorize it as a hand
                # '12' is arbitrary and based on image resolution captured from Pepper
                # this must be changed.  
                if len(keypoints) > 12:
                    output = "stop" 
                else :
                    output = "start" 

                print(output)
                # Send signal to Pepper with the state detected in the image
                WriteResultToDisk(output, state)
                state = output 

                # Hand detected, break out of loop to avoid starting if next object detected has to low confidence
                if state == "stop":
                    print("State is stop")
                    break

        # Archive the work file
        shutil.move(YOLOinput+filename2, archive+filename2)
        print("Moved " + YOLOinput+filename2 + " to " + archive+filename2)

        # If state was set to 'stop', wait 10 seconds before starting to look for hands again
        if state == "stop":
            print("Going to sleep")
            sleep(10)
            # Send start signal to Pepper, then contiune working
            WriteResultToDisk("start", state)

        counter = counter + 1
        print (counter)
        # Wait 1 second before continuing to work
        sleep(1)