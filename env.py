import pybullet as p
import pybullet_data
import numpy as np
import time
p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

huskypos = [0, -2, 1]

# Load URDFs
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])

target= p.loadURDF("block0.urdf",100,100, huskypos[2])

for i in range(500):
    p.stepSimulation()

time.sleep(2)
maxForce = 50 #Newton.m
#camera should be facing in the direction of the car
def get_positions_and_headings():
    rear_bump= p.getLinkState(husky,9)
    front_bump= p.getLinkState(husky,8)
    target_pos= p.getBasePositionAndOrientation(target)
    v1= front_bump[0][0]-rear_bump[0][0]
    v2= front_bump[0][1]-rear_bump[0][1]
    v3= front_bump[0][2]-rear_bump[0][2]
    frontVec= np.array(list((v1, v2, v3)))
    
    return (frontVec, target_pos, rear_bump)

def pid(bot_heading_vector, target_pos, vehicle_pos):
    """
    Inputs:
    bot_heading_vector - front vector of bot
    target_pos - position of the target box
    vehicle_pos - position of the vehicle

    Returns:
    linear - Linear speed of the bot
    angular - Angular speed of the bot

    Note: speed of right side wheels will be (linear + angular) and speed of left side wheels will be (linear - angular), you can refer to the turn function for more details
    """
    ###### code here and edit the linear and angular velocity
    ## Apply a P or PD controller


    #######################
    #    P CONTROLLER
    #######################

    
    i_bot= (bot_heading_vector[0]) ## i component of unit heading vector
    j_bot= (bot_heading_vector[1]) ## j component of unit heading vector

    x_tar= (target_pos[0][0])       # x co-ordinate of target
    y_tar= (target_pos[0][1])       # y co-ordinate of target

    x_bot= (vehicle_pos[0][0])      # x co-ordinate of position
    y_bot= (vehicle_pos[0][1])      # y co-ordinate of position

    i_req= (x_tar-x_bot)/(((x_tar-x_bot)**2)+((y_tar-y_bot)**2))**.5   # i component of unit vector of target-position line
    j_req= (y_tar-y_bot)/(((x_tar-x_bot)**2)+((y_tar-y_bot)**2))**.5   # j component of unit vector of target-position line


    
    angular = -50*(i_bot*j_req-j_bot*i_req)                #  P controller function of sin of angle between       

    linear= 2*(((x_tar-x_bot)**2)+((y_tar-y_bot)**2))**.5  # P controller fuction of distance between target pos and vehicle pos

    if(linear<3.6):   # safe distance to avoid collision = 1.8
        linear=0

    if(linear>20):    # max velocity to avoid topple of bot
        linear=20
    print(linear)

    return linear, angular

def turn(linear, angular):
    targetVel_R = linear + angular
    targetVel_L = linear - angular
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
    p.stepSimulation()



    
while (1):
    	
    bot_heading_vector, target_pos, vehicle_pos = get_positions_and_headings()
    linear, angular = pid(bot_heading_vector, target_pos, vehicle_pos)
    turn(linear, angular)
    time.sleep(0.02)
        
    
p.disconnect()