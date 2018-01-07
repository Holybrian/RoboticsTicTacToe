#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from al5d.srv import Ik
from random import randint
import imutils
import cv2
import numpy as np


#initialiseren camera
cam = cv2.VideoCapture(0)
cv2.namedWindow("window1")


#VERWERKEN VAN FOTO
def processImage(aantalZetten):
        # SPACE pressed
	# foto wordt getrokken en opgeslaan om verder in de code te verwerken
        img_name = "fieldImage.png"
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))

        # Load a color image in grayscale
        image = cv2.imread(img_name, 1)
        # define the list of boundaries
        boundaries = [
            ([60, 60, 125], [125, 125, 255], "Red"),
            ([150, 150, 150], [200, 200, 200], "Empty"),
            ([70, 125, 70], [125, 255, 125], "Green")
        ]
        Board = []
	
        # loop over the boundaries
        for (lower, upper, color) in boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask=mask)

            # use mask as proces image
            gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
            # find contours in the thresholded image
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]

            # loop over the contours
            for c in cnts:
                # compute the center of the contour
                square = cv2.boundingRect(c)

		# enkel contour tekenen voor areas die groter zijn dan 100 (breedte EN lengte)
                if square[2] > 100 and square[3] > 100:
                    M = cv2.moments(c)
                    if (M["m00"] == 0):
                        M["m00"] = 1
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # draw the contour and center of the shape on the image
                    cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                    cv2.circle(image, (cX, cY), 7, (0, 255, 0), -1)
                    cv2.putText(image, color + ", " + cX.__str__() + ", " + cY.__str__(), (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                    Board.append([cX.__str__(), cY.__str__(), color])


        # Order board
        def takeFirst(elem):
            return elem[0]


        Board.sort(key=takeFirst)


        def takeSecond(elem):
            return elem[1]


        Board.sort(key=takeSecond)


        vakken = []
        
	for i in Board:

	    if (len(vakken) == 0):
		vak = Vak(1, 1)
	    elif (len(vakken) == 1):
		vak = Vak(1, 2)
	    elif (len(vakken) == 2):
		vak = Vak(1, 3)
	    elif (len(vakken) == 3):
		vak = Vak(2, 1)
	    elif (len(vakken) == 4):
		vak = Vak(2, 2)
	    elif (len(vakken) == 5):
		vak = Vak(2, 3)
	    elif (len(vakken) == 6):
		vak = Vak(3, 1)
	    elif (len(vakken) == 7):
		vak = Vak(3, 2)
	    elif (len(vakken) == 8):
		vak = Vak(3, 3)

	    if (i[2] == "Empty"):
		vak.setVak(False, "")
		vakken.append(vak)
	    elif (i[2] == "Red"):
		vak.setVak(True, "O")
		vakken.append(vak)
	    elif (i[2] == "Green"):
		vak.setVak(True, "X")
		vakken.append(vak)

        print RobotBoard
	bord = Bord(vakken)
        # show the image show board
        print Board
        cv2.imshow("Image", image)

	spelgang(bord)


#KLASSEN
class Bord():
    aantal_vakken = 9
    def __init__(self, list_vakken):
        self.list_vakken = list_vakken

    def __str__(self):
        vak11 = self.list_vakken[0]
        vak12 = self.list_vakken[1]
        vak13 = self.list_vakken[2]
        vak21 = self.list_vakken[3]
        vak22 = self.list_vakken[4]
        vak23 = self.list_vakken[5]
        vak31 = self.list_vakken[6]
        vak32 = self.list_vakken[7]
        vak33 = self.list_vakken[8]
        return '-------\n' + '|{}|{}|{}|\n'.format(vak11.type_pion, vak12.type_pion, vak13.type_pion) + '-------\n' + '|{}|{}|{}|\n'.format(vak21.type_pion, vak22.type_pion, vak23.type_pion) + '-------\n' + '|{}|{}|{}|\n'.format(vak31.type_pion, vak32.type_pion, vak33.type_pion) + '-------\n'


class Vak():
    bezet = False
    type_pion = " "
    def __init__(self, coord_a, coord_b):
        self.coord_a = coord_a
        self.coord_b = coord_b

    def setVak(self, bezet, type):
        self.type_pion = type
        self.bezet = bezet

class Speler():
    def __init__(self, type_pion):
        self.typePion = type_pion

class BewegingBeurtRobot():
    def __init__(self, x, y, z, roll, gripper):
        rospy.init_node('joint_trajectory', anonymous=False)

        # Frame id and joints
        frame_id = "base"
        arm_joints = ['base_rotate',
                      'shoulder_tilt',
                      'elbow_tilt', 
                      'wrist_tilt',
                      'wrist_rotate',
                      'open_gripper']
        
        self.pos = Pose()
        self.pos.position.x = x
        self.pos.position.y = y
        self.pos.position.z = z
    	pitch = -1.45
    	roll = roll
    	joint_pos  = [0, 0, 0, 0, 0, gripper]
 
    	# Publisher to control the robot's joints
    	self.command = rospy.Publisher('/joint_controller/command', JointTrajectory, queue_size=1)    

    	rospy.wait_for_service("al5d_ik", 3)
    	self.IkService = rospy.ServiceProxy("al5d_ik", Ik)
    	response = self.IkService(self.pos, pitch, roll)
    	joint_pos[0] = response.j1
    	joint_pos[1] = response.j2
    	joint_pos[2] = response.j3
    	joint_pos[3] = response.j4
    	joint_pos[4] = response.j5
     
   	# Create a single-point arm trajectory with the joint_pos as the end-point
    	arm_trajectory = JointTrajectory()
    	arm_trajectory.joint_names = arm_joints
    	arm_trajectory.points.append(JointTrajectoryPoint())
    	arm_trajectory.points[0].positions = joint_pos
    	arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
    	arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
    	arm_trajectory.points[0].time_from_start = rospy.Duration(2.0)
    	arm_trajectory.header.frame_id = frame_id
    	arm_trajectory.header.stamp = rospy.Time.now()

    	# Publish trajectry
    	rospy.sleep(0.5)
    	self.command.publish(arm_trajectory)

    	rospy.loginfo('...done')


#FUNCTIONS

#BEURT SPELER
def beurtSpeler(speler, vak, bord):
    vak11 = bord.list_vakken[0]
    vak12 = bord.list_vakken[1]
    vak13 = bord.list_vakken[2]
    vak21 = bord.list_vakken[3]
    vak22 = bord.list_vakken[4]
    vak23 = bord.list_vakken[5]
    vak31 = bord.list_vakken[6]
    vak32 = bord.list_vakken[7]
    vak33 = bord.list_vakken[8]
    vak.bezet = True
    vak.type_pion = speler.typePion
    controle(speler, vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)


#KIJKEN WELKE MOGELIJKE ZETTEN ER ZIJN OF MOGELIJKE WINST VAN SPELER AFBLOKKEN
def beurtRobot(speler, bord):
    vak11 = bord.list_vakken[0]
    vak12 = bord.list_vakken[1]
    vak13 = bord.list_vakken[2]
    vak21 = bord.list_vakken[3]
    vak22 = bord.list_vakken[4]
    vak23 = bord.list_vakken[5]
    vak31 = bord.list_vakken[6]
    vak32 = bord.list_vakken[7]
    vak33 = bord.list_vakken[8]

    # MOGELIJKE WINST
    if ((vak11.bezet == True and vak11.type_pion == "O") and (vak12.bezet == True and vak12.type_pion == "O") and (vak13.bezet == False)):
        vak13.setVak(True, "O")
	beweging13()
    elif ((vak12.bezet == True and vak12.type_pion == "O") and (vak13.bezet == True and vak13.type_pion == "O") and (vak11.bezet == False)):
        vak11.setVak(True, "O")
	beweging11()
    elif ((vak11.bezet == True and vak11.type_pion == "O") and (vak21.bezet == True and vak21.type_pion == "O") and (vak31.bezet == False)):
        vak31.setVak(True, "O")
	beweging31()
    elif ((vak12.bezet == True and vak12.type_pion == "O") and (vak22.bezet == True and vak22.type_pion == "O") and (vak32.bezet == False)):
        vak32.setVak(True, "O")
	beweging32()
    elif ((vak13.bezet == True and vak13.type_pion == "O") and (vak23.bezet == True and vak23.type_pion == "O") and (vak33.bezet == False)):
        vak33.setVak(True, "O")
	beweging33()
    elif ((vak21.bezet == True and vak21.type_pion == "O") and (vak31.bezet == True and vak31.type_pion == "O") and (vak11.bezet == False)):
        vak11.setVak(True, "O")
	beweging11()
    elif ((vak22.bezet == True and vak22.type_pion == "O") and (vak32.bezet == True and vak32.type_pion == "O") and (vak21.bezet == False)):
        vak21.setVak(True, "O")
	beweging21()
    elif ((vak23.bezet == True and vak23.type_pion == "O") and (vak33.bezet == True and vak33.type_pion == "O") and (vak13.bezet == False)):
        vak13.setVak(True, "O")
	beweging13()
    elif ((vak21.bezet == True and vak21.type_pion == "O") and (vak22.bezet == True and vak22.type_pion == "O") and (vak23.bezet == False)):
        vak23.setVak(True, "O")
	beweging23()
    elif ((vak22.bezet == True and vak22.type_pion == "O") and (vak23.bezet == True and vak23.type_pion == "O") and (vak21.bezet == False)):
        vak21.setVak(True, "O")
	beweging21()
    elif ((vak31.bezet == True and vak31.type_pion == "O") and (vak32.bezet == True and vak32.type_pion == "O") and (vak33.bezet == False)):
        vak33.setVak(True, "O")
	beweging33()
    elif ((vak32.bezet == True and vak32.type_pion == "O") and (vak33.bezet == True and vak33.type_pion == "O") and (vak31.bezet == False)):
        vak31.setVak(True, "O")
	beweging31()
    elif ((vak11.bezet == True and vak11.type_pion == "O") and (vak13.bezet == True and vak13.type_pion == "O") and (vak12.bezet == False)):
        vak12.setVak(True, "O")
	beweging12()
    elif ((vak21.bezet == True and vak21.type_pion == "O") and (vak23.bezet == True and vak23.type_pion == "O") and (vak22.bezet == False)):
        vak22.setVak(True, "O")
	beweging22()
    elif ((vak31.bezet == True and vak31.type_pion == "O") and (vak33.bezet == True and vak33.type_pion == "O") and (vak32.bezet == False)):
        vak32.setVak(True, "O")
	beweging32()
    elif ((vak31.bezet == True and vak31.type_pion == "O") and (vak13.bezet == True and vak13.type_pion == "O") and (vak22.bezet == False)):
        vak22.setVak(True, "O")
	beweging22()
    elif ((vak11.bezet == True and vak11.type_pion == "O") and (vak33.bezet == True and vak33.type_pion == "O") and (vak22.bezet == False)):
        vak22.setVak(True, "O")
	beweging22()
    elif ((vak11.bezet == True and vak11.type_pion == "O") and (vak31.bezet == True and vak31.type_pion == "O") and (vak21.bezet == False)):
        vak21.setVak(True, "O")
	beweging21()
    elif ((vak12.bezet == True and vak12.type_pion == "O") and (vak32.bezet == True and vak32.type_pion == "O") and (vak22.bezet == False)):
        vak22.setVak(True, "O")
	beweging22()
    elif ((vak13.bezet == True and vak13.type_pion == "O") and (vak33.bezet == True and vak33.type_pion == "O") and (vak23.bezet == False)):
        vak23.setVak(True, "O")
	beweging23()

    #ZETTEN AFBLOKKEN
    elif ((vak11.bezet == True and vak11.type_pion == "X") and (vak12.bezet == True and vak12.type_pion == "X")):
        if (vak13.bezet == False):
            vak13.setVak(True, "O")
	    beweging13()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak12.bezet == True and vak12.type_pion == "X") and (vak13.bezet == True and vak13.type_pion == "X")):
        if (vak11.bezet == False):
            vak11.setVak(True, "O")
	    beweging11()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak11.bezet == True and vak11.type_pion == "X") and (vak21.bezet == True and vak21.type_pion == "X")):
        if (vak31.bezet == False):
            vak31.setVak(True, "O")
	    beweging31()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak12.bezet == True and vak12.type_pion == "X") and (vak22.bezet == True and vak22.type_pion == "X")):
        if (vak32.bezet == False):
            vak32.setVak(True, "O")
	    beweging32()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak13.bezet == True and vak13.type_pion == "X") and (vak23.bezet == True and vak23.type_pion == "X")):
        if (vak33.bezet == False):
            vak33.setVak(True, "O")
	    beweging33()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak21.bezet == True and vak21.type_pion == "X") and (vak31.bezet == True and vak31.type_pion == "X")):
        if (vak11.bezet == False):
            vak11.setVak(True, "O")
	    beweging11()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak22.bezet == True and vak22.type_pion == "X") and (vak32.bezet == True and vak32.type_pion == "X")):
        if (vak21.bezet == False):
            vak21.setVak(True, "O")
	    beweging21()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak23.bezet == True and vak23.type_pion == "X") and (vak33.bezet == True and vak33.type_pion == "X")):
        if (vak13.bezet == False):
            vak13.setVak(True, "O")
	    beweging13()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak21.bezet == True and vak21.type_pion == "X") and (vak22.bezet == True and vak22.type_pion == "X")):
        if (vak23.bezet == False):
            vak23.setVak(True, "O")
	    beweging23()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak22.bezet == True and vak22.type_pion == "X") and (vak23.bezet == True and vak23.type_pion == "X")):
        if (vak21.bezet == False):
            vak21.setVak(True, "O")
	    beweging21()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak31.bezet == True and vak31.type_pion == "X") and (vak32.bezet == True and vak32.type_pion == "X")):
        if (vak33.bezet == False):
            vak33.setVak(True, "O")
	    beweging33()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak32.bezet == True and vak32.type_pion == "X") and (vak33.bezet == True and vak33.type_pion == "X")):
        if (vak31.bezet == False):
            vak31.setVak(True, "O")
	    beweging31()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak11.bezet == True and vak11.type_pion == "X") and (vak22.bezet == True and vak22.type_pion == "X")):
        if (vak33.bezet == False):
            vak33.setVak(True, "O")
	    beweging33()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak22.bezet == True and vak22.type_pion == "X") and (vak33.bezet == True and vak33.type_pion == "X")):
        if (vak11.bezet == False):
            vak11.setVak(True, "O")
	    beweging11()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak31.bezet == True and vak31.type_pion == "X") and (vak22.bezet == True and vak22.type_pion == "X")):
        if (vak13.bezet == False):
            vak13.setVak(True, "O")
	    beweging13()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak13.bezet == True and vak13.type_pion == "X") and (vak22.bezet == True and vak22.type_pion == "X")):
        if (vak31.bezet == False):
            vak31.setVak(True, "O")
	    beweging31()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak11.bezet == True and vak11.type_pion == "X") and (vak13.bezet == True and vak13.type_pion == "X")):
        if (vak12.bezet == False):
            vak12.setVak(True, "O")
	    beweging12()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak21.bezet == True and vak21.type_pion == "X") and (vak23.bezet == True and vak23.type_pion == "X")):
        if (vak22.bezet == False):
            vak22.setVak(True, "O")
	    beweging22()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak31.bezet == True and vak31.type_pion == "X") and (vak33.bezet == True and vak33.type_pion == "X")):
        if (vak32.bezet == False):
            vak32.setVak(True, "O")
	    beweging32()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak31.bezet == True and vak31.type_pion == "X") and (vak13.bezet == True and vak13.type_pion == "X")):
        if (vak22.bezet == False):
            vak22.setVak(True, "O")
	    beweging22()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak11.bezet == True and vak11.type_pion == "X") and (vak33.bezet == True and vak33.type_pion == "X")):
        if (vak22.bezet == False):
            vak22.setVak(True, "O")
	    beweging22()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak11.bezet == True and vak11.type_pion == "X") and (vak31.bezet == True and vak31.type_pion == "X")):
        if (vak21.bezet == False):
            vak21.setVak(True, "O")
	    beweging21()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak12.bezet == True and vak12.type_pion == "X") and (vak32.bezet == True and vak32.type_pion == "X")):
        if (vak22.bezet == False):
            vak22.setVak(True, "O")
	    beweging22()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    elif ((vak13.bezet == True and vak13.type_pion == "X") and (vak33.bezet == True and vak33.type_pion == "X")):
        if (vak23.bezet == False):
            vak23.setVak(True, "O")
	    beweging23()
        else:
            randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    #RIJEN AANVULLEN
    #11
    elif((vak11.bezet == True and vak11.type_pion == "O") and vak12.bezet == False and vak13.bezet == False):
        vak12.setVak(True, "O")
	beweging12()
    elif ((vak11.bezet == True and vak11.type_pion == "O") and vak21.bezet == False and vak31.bezet == False):
        vak21.setVak(True, "O")
	beweging21()
    elif ((vak11.bezet == True and vak11.type_pion == "O") and vak22.bezet == False and vak33.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    #12
    elif ((vak12.bezet == True and vak12.type_pion == "O") and vak11.bezet == False and vak13.bezet == False):
        vak11.setVak(True, "O")
	beweging11()
    elif ((vak12.bezet == True and vak12.type_pion == "O") and vak22.bezet == False and vak32.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    #13
    elif ((vak13.bezet == True and vak13.type_pion == "O") and vak12.bezet == False and vak11.bezet == False):
        vak12.setVak(True, "O")
	beweging12()
    elif ((vak13.bezet == True and vak13.type_pion == "O") and vak23.bezet == False and vak33.bezet == False):
        vak23.setVak(True, "O")
	beweging23()
    elif ((vak13.bezet == True and vak13.type_pion == "O") and vak22.bezet == False and vak31.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    #21
    elif ((vak21.bezet == True and vak21.type_pion == "O") and vak22.bezet == False and vak23.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    elif ((vak21.bezet == True and vak21.type_pion == "O") and vak11.bezet == False and vak31.bezet == False):
        vak11.setVak(True, "O")
	beweging11()
    #22
    elif ((vak22.bezet == True and vak22.type_pion == "O") and vak11.bezet == False and vak13.bezet == False):
        vak11.setVak(True, "O")
	beweging11()
    elif ((vak22.bezet == True and vak22.type_pion == "O") and vak12.bezet == False and vak32.bezet == False):
        vak12.setVak(True, "O")
	beweging12()
    elif ((vak22.bezet == True and vak22.type_pion == "O") and vak11.bezet == False and vak33.bezet == False):
        vak11.setVak(True, "O")
	beweging11()
    #23
    elif ((vak23.bezet == True and vak23.type_pion == "O") and vak22.bezet == False and vak21.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    elif ((vak23.bezet == True and vak23.type_pion == "O") and vak13.bezet == False and vak33.bezet == False):
        vak13.setVak(True, "O")
	beweging13()
    #31
    elif ((vak31.bezet == True and vak31.type_pion == "O") and vak32.bezet == False and vak33.bezet == False):
        vak32.setVak(True, "O")
	beweging32()
    elif ((vak31.bezet == True and vak31.type_pion == "O") and vak21.bezet == False and vak11.bezet == False):
        vak21.setVak(True, "O")
	beweging21()
    elif ((vak31.bezet == True and vak31.type_pion == "O") and vak22.bezet == False and vak11.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    #32
    elif ((vak32.bezet == True and vak32.type_pion == "O") and vak31.bezet == False and vak33.bezet == False):
        vak31.setVak(True, "O")
	beweging31()
    elif ((vak32.bezet == True and vak32.type_pion == "O") and vak22.bezet == False and vak12.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    #33
    elif ((vak33.bezet == True and vak33.type_pion == "O") and vak32.bezet == False and vak31.bezet == False):
        vak32.setVak(True, "O")
	beweging32()
    elif ((vak33.bezet == True and vak33.type_pion == "O") and vak23.bezet == False and vak13.bezet == False):
        vak23.setVak(True, "O")
	beweging23()
    elif ((vak33.bezet == True and vak33.type_pion == "O") and vak22.bezet == False and vak11.bezet == False):
        vak22.setVak(True, "O")
	beweging22()
    #RANDOM ZET
    else:
        randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)
    controle(speler, vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33)

#RANDOM ZET ROBOT
def randomZet(vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33):
    zetGekozen = False
    while (zetGekozen == False):
        a = randint(1, 3)
        b = randint(1, 3)
        if ((a == 1) and (b == 1)):
            if (vak11.bezet == False):
                vak11.setVak(True, "O")
                zetGekozen = True
		beweging11()
        if ((a == 1) and (b == 2)):
            if (vak12.bezet == False):
                vak12.setVak(True, "O")
                zetGekozen = True
		beweging12()
        if ((a == 1) and (b == 3)):
            if (vak13.bezet == False):
                vak13.setVak(True, "O")
                zetGekozen = True
		beweging13()
        if ((a == 2) and (b == 1)):
            if (vak21.bezet == False):
                vak21.setVak(True, "O")
                zetGekozen = True
		beweging21()
        if ((a == 2) and (b == 2)):
            if (vak22.bezet == False):
                vak22.setVak(True, "O")
                zetGekozen = True
		beweging22()
        if ((a == 2) and (b == 3)):
            if (vak23.bezet == False):
                vak23.setVak(True, "O")
                zetGekozen = True
		beweging23()
        if ((a == 3) and (b == 1)):
            if (vak31.bezet == False):
                vak31.setVak(True, "O")
                zetGekozen = True
		beweging31()
        if ((a == 3) and (b == 2)):
            if (vak32.bezet == False):
                vak32.setVak(True, "O")
                zetGekozen = True
		beweging32()
        if ((a == 3) and (b == 3)):
            if (vak33.bezet == False):
                vak33.setVak(True, "O")
                zetGekozen = True
		beweging33()

#CONTROLE 3 OP EEN RIJ
def controle(speler, vak11, vak12, vak13, vak21, vak22, vak23, vak31, vak32, vak33):
    if ((vak11.bezet == True and vak11.type_pion == speler.typePion) and (vak12.bezet == True and vak12.type_pion == speler.typePion) and (vak13.bezet == True and vak13.type_pion == speler.typePion)):
        afsluit()
    elif ((vak21.bezet == True and vak21.type_pion == speler.typePion) and (vak22.bezet == True and vak22.type_pion == speler.typePion) and (vak23.bezet == True and vak23.type_pion == speler.typePion)):
        afsluit()
    elif ((vak31.bezet == True and vak31.type_pion == speler.typePion) and (vak32.bezet == True and vak32.type_pion == speler.typePion) and (vak33.bezet == True and vak33.type_pion == speler.typePion)):
        afsluit()
    elif ((vak11.bezet == True and vak11.type_pion == speler.typePion) and (vak21.bezet == True and vak21.type_pion == speler.typePion) and (vak31.bezet == True and vak31.type_pion == speler.typePion)):
        afsluit()
    elif ((vak12.bezet == True and vak12.type_pion == speler.typePion) and (vak22.bezet == True and vak22.type_pion == speler.typePion) and (vak32.bezet == True and vak32.type_pion == speler.typePion)):
        afsluit()
    elif ((vak13.bezet == True and vak13.type_pion == speler.typePion) and (vak23.bezet == True and vak23.type_pion == speler.typePion) and (vak33.bezet == True and vak33.type_pion == speler.typePion)):
        afsluit()
    elif ((vak11.bezet == True and vak11.type_pion == speler.typePion) and (vak22.bezet == True and vak22.type_pion == speler.typePion) and (vak33.bezet == True and vak33.type_pion == speler.typePion)):
        afsluit()
    elif ((vak13.bezet == True and vak13.type_pion == speler.typePion) and (vak22.bezet == True and vak22.type_pion == speler.typePion) and (vak31.bezet == True and vak31.type_pion == speler.typePion)):
        afsluit()

#AFSLUIT VAN HET SPEL EN HEROPSTARTEN
def afsluit():
    #CODE VOOR BORD OP TE RUIMEN
    spelgang()

def beweging11():
    BewegingBeurtRobot(0.19, 0.1, 0.1, 0.45, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.19, 0.1, 0.0125, 0.45, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.19, 0.1, 0.0125, 0.45, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.19, 0.1, 0.1, 0.45, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging12():
    BewegingBeurtRobot(0.146, 0.105, 0.1, 0.6, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.146, 0.105, 0.0125, 0.6, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.146, 0.105, 0.0125, 0.6, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.146, 0.105, 0.1, 0.6, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging13():
    BewegingBeurtRobot(0.1, 0.11, 0.1, 0.8, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.11, 0.015, 0.8, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.11, 0.015, 0.8, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.11, 0.1, 0.8, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging21():
    BewegingBeurtRobot(0.191, 0.146, 0.1, 0.6, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.191, 0.146, 0.0125, 0.6, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.191, 0.146, 0.0125, 0.6, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.191, 0.146, 0.1, 0.6, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging22():
    BewegingBeurtRobot(0.146, 0.15, 0.1, 0.8, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.146, 0.15, 0.0125, 0.8, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.146, 0.15, 0.0125, 0.8, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.146, 0.15, 0.1, 0.8, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging23():
    BewegingBeurtRobot(0.1, 0.155, 0.1, 1, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.155, 0.0125, 1, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.155, 0.0125, 1, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.155, 0.1, 1, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging31():
    BewegingBeurtRobot(0.202, 0.096, 0.1, 0.75, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.202, 0.096, 0.0250, 0.75, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.202, 0.096, 0.0250, 0.75, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.202, 0.096, 0.1, 0.75, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging32():
    BewegingBeurtRobot(0.155, 0.196, 0.1, 0.95, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.155, 0.196, 0.024, 0.95, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.155, 0.196, 0.024, 0.95, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.155, 0.196, 0.1, 0.95, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def beweging33():
    BewegingBeurtRobot(0.1, 0.2, 0.1, 1.1, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.2, 0.015, 1.1, 0.5)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.2, 0.015, 1.1, 2)
    time.sleep(2)
    BewegingBeurtRobot(0.1, 0.2, 0.1, 1.1, 2)
    time.sleep(2)
    bewegingStart()
    time.sleep(2)

def neemPion(nummer):
    if (nummer == 1):
	BewegingBeurtRobot(0.205, 0.005, 0.1, 0, 2)
	time.sleep(2)
    	BewegingBeurtRobot(0.205, 0.005, 0.01, 0, 2)
	time.sleep(2)
    	BewegingBeurtRobot(0.205, 0.005, 0.01, 0, 0.5)
    	time.sleep(2)
    	BewegingBeurtRobot(0.205, 0.005, 0.1, 0, 0.5)
    	time.sleep(2)
    elif (nummer == 2):
	BewegingBeurtRobot(0.125, 0.005, 0.1, 0, 2)
    	time.sleep(2)
    	BewegingBeurtRobot(0.125, 0.005, 0.02, 0, 2)
    	time.sleep(2)
    	BewegingBeurtRobot(0.125, 0.005, 0.02, 0, 0.5)
    	time.sleep(2)
    	BewegingBeurtRobot(0.125, 0.005, 0.1, 0, 0.5)
    	time.sleep(2)
    elif (nummer == 3):
	BewegingBeurtRobot(0.205, -0.105, 0.1, -0.35, 2)
    	time.sleep(2)
    	BewegingBeurtRobot(0.205, -0.105, 0.02, -0.35, 2)
    	time.sleep(2)
    	BewegingBeurtRobot(0.205, -0.105, 0.02, -0.35, 0.5)
    	time.sleep(2)
    	BewegingBeurtRobot(0.205, -0.105, 0.1, -0.35, 0.5)
    	time.sleep(2)
    elif (nummer == 4):
	BewegingBeurtRobot(0.165, -0.050, 0.1, -0.35, 2)
    	time.sleep(2)
    	BewegingBeurtRobot(0.165, -0.050, 0.02, -0.35, 2)
    	time.sleep(2)
    	BewegingBeurtRobot(0.165, -0.050, 0.02, -0.35, 0.5)
    	time.sleep(2)
    	BewegingBeurtRobot(0.165, -0.050, 0.1, -0.35, 0.5)
    	time.sleep(2)

def bewegingStart():
    BewegingBeurtRobot(0.2, -0.1, 0.1, 0, 2)
    

#GANG VAN HET SPEL
def spelgang(bord):
    speler = Speler("X")
    robot = Speler("O")

    while (aantalZetten <= 4):

	neemPion(aantalZetten)
        beurtRobot(robot, bord)

        aantalZetten += 1
        print(bord)


#INFINITE LOOP (blijft lopen totdat ESC is ingeduwd)
while True:
    aantalZetten = 1
    ret, frame = cam.read()
    cv2.imshow("window1", frame)
    if not ret:
	break
    k = cv2.waitKey(1)

    if k % 256 == 27:
	# ESC pressed
	print("Escape hit, closing...")
	break
    elif k % 256 == 32:
	processImage(aantalZetten)
	aantalZetten += 1

#camera afsluiten
cam.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
