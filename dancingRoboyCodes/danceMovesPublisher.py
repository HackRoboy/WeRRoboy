#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import Point
from roboy_communication_simulation.msg import DanceMoves

def publishDanceMoves():
	pub = rospy.Publisher('/roboy/DanceMoves', DanceMoves,  queue_size=10)
	rospy.init_node('danceMovesPublisher', anonymous=True)
	rate = rospy.Rate(2) # hz
	
	#### Create Points
	pointLeftArm = Point()
	pointRightArm = Point()
	pointLeftLeg = Point()
	pointRightLeg = Point()

        lefArmArrayx = np.array([10, -10, 3, -3, 20])
        lefArmArrayy = np.array([-10, 10, -2, 2, -20])
        lefArmArrayz = np.array([-10, 10, -1, 1, 0])

        rightArmArrayx = np.array([10, -10, 3, -3, 7])
        rightArmArrayy = np.array([-20, 20, -2, 2, -6])
        rightArmArrayz = np.array([10, -10, -1, 1, 0])

        leftLegArrayx = np.array([2, -2, 5, -1, 3])
        leftLegArrayy = np.array([0.3, -0.3, 1, -1, 1])
        leftLegArrayz = np.array([0.2, 2, 0.2, 1, -0.3])
        
        rightLegArrayx = np.array([-20, 20, 5, -5, 30])
        rightLegArrayy = np.array([10, -10, 5, -5, 30])
        rightLegArrayz = np.array([100, 100,100, 100])


        #### Create DanceMoves msg
        msg = DanceMoves()

        while not rospy.is_shutdown():
	        #### Hard-Code Each Points
                for i in range(0,2):
	                pointLeftArm.x = lefArmArrayx[i]
	                pointLeftArm.y = lefArmArrayy[i]
	                pointLeftArm.z = lefArmArrayz[i]

	                pointRightArm.x = rightArmArrayx[i]
	                pointRightArm.y = rightArmArrayy[i]
	                pointRightArm.z = rightArmArrayz[i]


	                pointLeftLeg.x = leftLegArrayx[i]
	                pointLeftLeg.y = leftLegArrayy[i]
	                pointLeftLeg.z = leftLegArrayz[i]

	
	                pointRightLeg.x = rightLegArrayx[i]
	                pointRightLeg.y = rightLegArrayy[i]
	                pointRightLeg.z = rightLegArrayz[i]
                        
                	msg.leftArm = pointLeftArm
	                msg.rightArm = pointRightArm
	                msg.leftLeg = pointLeftLeg
	                msg.rightLeg = 	pointRightLeg
                        
                        rospy.loginfo(msg)
                        pub.publish(msg)
                        rate.sleep()
                #rate.sleep()
	
	#### Publish DanceMoves msg
	#while not rospy.is_shutdown():
	#	rospy.loginfo(msg)
	#	pub.publish(msg)
	#	rate.sleep()



if __name__ == '__main__':
	try:
		publishDanceMoves()
	except rospy.ROSInterruptException:
		pass
