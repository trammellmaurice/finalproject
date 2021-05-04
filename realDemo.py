import math, rospy
from turtleAPI import robot
import cv2
import numpy as np

# make the robot and lists for tracking error
r = robot()
error_list = []
pi = math.pi

# pid
def pid_speed(kp, ki, kd, error, old_error, error_list):

    # add the error to the integral portion
    if len(error_list) > 5:
        error_list.pop()
        error_list.append(error)

    # calculate sum
    error_sum = 0
    for i in error_list:
        error_sum += i

    # kp portion + ki portion
    to_return = (kp * error) + (ki * error_sum)
    to_return += kd * (error - old_error)

    return to_return

def hunt(color):
    speed_limit = .15 # speed limit
    turn_limit = .4 # turn speed limit
    colormap = {"blue":[220,240],"green":[130,160],"purple":[260,305],"red":[0,10],"yellow":[50,70]}
    bot = np.array([colormap[color][0]/2, 70, 70])
    top = np.array([colormap[color][1]/2,235,235])

    rate = rospy.Rate(5)

    #size = spin(r,top,bot,turn_limit,rate)
    size = 1000
    
    r.drive(angSpeed=0, linSpeed=0)
    #print("done with spin")
    #print("largest blob: ",size)

    # loop until at position
    old_error = 0

    #used to track which way to turn
    last_seen = 1 # 1 for left, -1 for right

    while not rospy.is_shutdown():
        time = 0
        image = r.getImage()
        height, width = image.shape[0:2]
        image = image[height/3:2*height/3,width/3:2*width/3]
        #cv2.imshow('normal',image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mapimage = cv2.inRange(hsv, bot, top)

        

        augimage = image
        augimage[:, :, 1] = np.bitwise_or(image[:, :, 1], mapimage)
        cv2.imshow('augmented',augimage)
        cv2.waitKey(1)
        # current pos
        current_pos = r.getPositionTup()
        current_angle = current_pos[2]
        
        height, width = mapimage.shape[0:2]
        total = cv2.countNonZero(mapimage)
        #print("total: " +str(total))
        width = width//2 # integer division
        halfLeft = mapimage[:,:width]
        left = cv2.countNonZero(halfLeft)
        halfRight = mapimage[:,width:]
        right = cv2.countNonZero(halfRight)
        #print ("total: "+str(total)+" left: "+str(left)+" right: "+str(right))

        # calculate angle speed and lin speed drive
        lin_speed = speed_limit
        
        # if less than 1/3rd of the original blob size found keep turning left
        # but with 0 lin_speed
        if (total < size/3):
            error = 1000 * last_seen
            lin_speed = 0
            #print("Can't see")
        else:
            error = (left - right)/float(total/10)
            print("left: "+str(left)+" right: "+str(right)+" total: "+str(total))
            last_seen = np.sign(error)
            print("last seen: "+str(last_seen)) 

        
        dpth=r.getDepth()
        height, width = dpth.shape[0:2]
        middle_row = dpth[height/2,:]
        #print(middle_row)
        if len(middle_row[np.nonzero(middle_row)]) > 0:
            min = np.nanmin(middle_row[np.nonzero(middle_row)])
        else:
            min = 1200
        #print (min)
        if (min < 500): # FIX THIS LINE
            lin_speed = 0
            if (abs(error) < 10):
                r.drive(angSpeed=0, linSpeed=0)
                print("done")
                return
        
        # speed
        ang_speed = pid_speed(.2, .01, .001, error, old_error, error_list)
        #ang_speed = pid_speed(.2, 0, 0, error, old_error, error_list)

        #ang_speed = ang_speed + (lin_speed/speed_limit)*np.sign(ang_speed)

        if (ang_speed > turn_limit):
            ang_speed = turn_limit
        elif (ang_speed < -turn_limit):
            ang_speed = -turn_limit

        r.drive(angSpeed=ang_speed, linSpeed=lin_speed)
        #print('speed: ' + str(ang_speed) + ' ' + str(lin_speed))

        # set old values
        old_error=error
        rate.sleep()


def Blob(): 
    # https://learnopencv.com/blob-detection-using-opencv-python-c/
    # https://www.geeksforgeeks.org/find-circles-and-ellipses-in-an-image-using-opencv-python/
    # https://stackoverflow.com/questions/42203898/python-opencv-blob-detection-or-circle-detection
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    
    # Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1500

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.5

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else :
        detector = cv2.SimpleBlobDetector_create(params)

    blank = np.zeros((1, 1)) 
    blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),
                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
  
    number_of_blobs = len(keypoints)
    text = "Number of Circular Blobs: " + str(len(keypoints))
    cv2.putText(blobs, text, (20, 550),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2)
  
    # Show blobs
    cv2.imshow("Filtering Circular Blobs Only", blobs)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#color = raw_input("What color to hunt:")
"""color = "purple"
hunt(color)"""
