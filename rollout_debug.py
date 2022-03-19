import airsim
import cv2
import numpy as np
import time
import tensorflow as tf
import os
from agil_gaze import my_softmax, my_kld
from agil_airsim import agilNN
import math

# connect to AirSim client
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

def _toEulerianAngle(q):
        z = q.z_val
        y = q.y_val
        x = q.x_val
        w = q.w_val
        ysqr = y * y

        # roll (x-axis rotation)
        t0 = +2.0 * (w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + ysqr)
        roll = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w*y - z*x)
        if (t2 > 1.0):
            t2 = 1
        if (t2 < -1.0):
            t2 = -1.0
        pitch = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w*z + x*y)
        t4 = +1.0 - 2.0 * (ysqr + z*z)
        yaw = math.atan2(t3, t4)

        return (pitch, roll, yaw)

customObjects = {
    'my_softmax': my_softmax,
    'my_kld': my_kld,
}

agil_model = "agil_model.h5"
gaze_model = "gaze.h5"
gaze = tf.keras.models.load_model(gaze_model, custom_objects=customObjects)
agil = tf.keras.models.load_model(agil_model)

# rollout loop
img_counter = 0
airsim.wait_key('Press any key to begin rollouts')
while(True):

    # arm and takeoff
    print("Taking off...")
    client.armDisarm(True)
    client.takeoffAsync().join()

    # just hover
    client.hoverAsync().join()

    # get the image of the scene
    sc = 10
    duration = 1e-1
    done = False
    while(not done):
        # getting quad states
        state = client.getMultirotorState()

        # convert from quaternion to euler angles
        (pitch, roll, yaw) = _toEulerianAngle(state.kinematics_estimated.orientation)
        # getting images
        kairos = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        kairos_bstr = kairos[0]

        # get numpy array
        img1d = np.fromstring(kairos_bstr.image_data_uint8, dtype=np.uint8)

        # reshape to 4-channel image (for Unreal 4.18)
        #img_rgb = img1d.reshape(kairos_bstr.height, kairos_bstr.width, 4)
        
        # reshape to 3-channel image (for Unreal 4.25)
        img_rgb = img1d.reshape(kairos_bstr.height, kairos_bstr.width, 3)

        # for Unreal 4.18
        # img = img_rgb[:,:,0:3]
        
        # for Unreal 4.25
        img = img_rgb

        cv2.imwrite('out{}.png'.format(img_counter), img)
        # AGIL network predictions
        # roll, pitch, throttle, yaw
        output = agilNN(gaze, agil, img)
        act_roll     = float(output[:,0]) 
        act_pitch    = float(output[:,1]) 
        #throttle = abs(float(output[:,2]))

        act_throttle = (float(output[:,2]) + 1.0)/2.0 + .15
        act_yaw      = float(output[:,3]) 
        # print(roll, pitch, throttle, yaw)

        ################## I am not sure about the following############################
        vx = sc / 1.5 * act_pitch
        vy = sc / 1.5 * act_roll
        vz = 10 * sc * act_yaw
        ref_alt = state.kinematics_estimated.position.z_val + sc / 2 * act_throttle

        # translate from inertial to body frame
        C = np.zeros((2, 2))
        C[0, 0] = np.cos(yaw)
        C[0, 1] = -np.sin(yaw)
        C[1, 0] = -C[0, 1]
        C[1, 1] = C[0, 0]
        vb = C.dot(np.array([vx, vy]))
        ###############################################################################
        print(vb[0], vb[1], ref_alt, duration)
     
        # # actions mapped to controlling the quadrotor; args := (roll, pitch, yaw, throttle, duration)
        # client.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, 1)

        # send commands
        client.moveByVelocityZAsync(
            vb[0],
            vb[1],
            -ref_alt,
            duration,
            airsim.DrivetrainType.MaxDegreeOfFreedom,
            airsim.YawMode(True, vz),
        )
             
        # definition of episode completion (for now, I just wrote hacky done condition that ends each episode after two minutes)
        #if(time.time() > time.time() + 60*2):
        #    done = True
    
        img_counter = img_counter + 1
    # resets at the end of the episode
    client.reset()
