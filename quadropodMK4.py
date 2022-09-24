# imports for this class_______________
import numpy as np
import time
import pygame
import math

#from robot_state_display import robot_state_display


# Important servo controll classes from adafruit libraries
from adafruit_servokit import ServoKit;
import adafruit_motor.servo;

class quadropodMK4:
    def __init__(self,i2c_address:'int' = 0x40,instant_control:'logical'=False):
            self.i2c_address = i2c_address;        
            self.kit = ServoKit(channels=16,address = i2c_address);
            
            # initial state servo angles
            self.servo0angle = 90;
            self.servo1angle = 90;
            self.servo2angle = 90;
            self.servo3angle = 90;
            self.servo4angle = 90;
            self.servo5angle = 90;
            self.servo6angle = 90;
            self.servo7angle = 90;
            
            # Define servo angle ranges or the HARD Constarints
            self.servo0_min = 0;self.servo0_max = 180;
            self.servo4_min = 0;self.servo4_max = 180;
            
            self.servo1_min = 0;self.servo1_max = 180;
            self.servo5_min = 0;self.servo5_max = 180;
            
            self.servo2_min = 0;self.servo2_max = 180;
            self.servo6_min = 0;self.servo6_max = 180;
            
            self.servo3_min = 0;self.servo3_max = 180;
            self.servo7_min = 0;self.servo7_max = 180;
            
            # Robot state vectors
            self.state_vector_leanBackward = [120, 60, 60, 120, 90, 90, 120, 60];
            self.state_vector_leanForward = [120, 60, 60, 120, 60, 120, 90, 90];
            self.state_vector_leanLeft = [110, 90, 90, 110, 70, 70, 70, 70];
            self.state_vector_leanRight = [90, 70, 70, 90, 110, 110, 110, 110]
            
            # Robots walking state vectors
            self.state_vector_walk_init = [90, 90, 90, 90, 120, 60, 60, 120];
    
    def pass_servo_cmd(self,angle:'float'=90,servo_num:'int'=0,powerMode:'bool'=False):
        # Function to control the servo in asafe way
        if servo_num == 0 and angle<=self.servo0_max and angle>=self.servo0_min:
            self.kit.servo[0].angle = angle;
            self.servo0angle = angle;            
            print('-> SERVO 0 moved to position:',angle);
            #return angle;
        elif servo_num == 4 and angle<=self.servo4_max and angle>=self.servo4_min:
            self.kit.servo[4].angle = angle;
            self.servo4angle = angle;
            print('-> SERVO 4 moved to position:',angle);
            #return angle;
            
        elif servo_num == 1 and angle<=self.servo1_max and angle>=self.servo1_min:
            self.kit.servo[1].angle = angle;
            self.servo1angle = angle;
            print('-> SERVO 1 moved to position:',angle);
            #return angle;
        elif servo_num == 5 and angle<=self.servo5_max and angle>=self.servo5_min:
            self.kit.servo[5].angle = angle;
            self.servo5angle = angle;
            print('-> SERVO 5 moved to position:',angle);
            #return angle;
            
        elif servo_num == 2 and angle<=self.servo2_max and angle>=self.servo2_min:
            self.kit.servo[2].angle = angle;
            self.servo2angle = angle;
            print('-> SERVO 2 moved to position:',angle);
            #return angle;
        elif servo_num == 6 and angle<=self.servo6_max and angle>=self.servo6_min:
            self.kit.servo[6].angle = angle;
            self.servo6angle = angle;
            print('-> SERVO 6 moved to position:',angle);
            #return angle;
            
        elif servo_num == 3 and angle<=self.servo3_max and angle>=self.servo3_min:
            self.kit.servo[3].angle = angle;
            self.servo3angle = angle;
            print('-> SERVO 3 moved to position:',angle);
            #return angle;
        elif servo_num == 7 and angle<=self.servo7_max and angle>=self.servo7_min:
            self.kit.servo[7].angle = angle;
            self.servo7angle = angle;
            print('-> SERVO 7 moved to position:',angle);
            #return angle;
            
        else:
            print('-> WARNING: SERVO or ANGLE out of range');
            #return None;
        #return angle    
            
        if(powerMode): # Sets all pwm signals to OFF to reduce load
            time.sleep(0.01);
            self.kit.servo[0].angle = None;
            self.kit.servo[1].angle = None;
            self.kit.servo[2].angle = None;
            self.kit.servo[3].angle = None;
            self.kit.servo[4].angle = None;
            self.kit.servo[5].angle = None;
            self.kit.servo[6].angle = None;
            self.kit.servo[7].angle = None;
            
            
    def robot_stand(self,dev_angle:int=40,powerMode:bool=True):
        #dev_angle = 00;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90+dev_angle,4,powerMode);
        self.pass_servo_cmd(90-dev_angle,5,powerMode);
        self.pass_servo_cmd(90-dev_angle,6,powerMode);
        self.pass_servo_cmd(90+dev_angle,7,powerMode);
        self.robot_stand_state = self.get_state();
        

    def robot_sit(self,dev_angle:int=40,powerMode:bool=True):
        #dev_angle = 00;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90-dev_angle,4,powerMode);
        self.pass_servo_cmd(90+dev_angle,5,powerMode);
        self.pass_servo_cmd(90+dev_angle,6,powerMode);
        self.pass_servo_cmd(90-dev_angle,7,powerMode);
        self.robot_sit_state = self.get_state();
    
    def move_leg_in_sine(self,leg:int=0,state:list=[90,90],step:float=0.2):
        pass
    
    ## Inividual leg movement commands keeping others at stand position ######
    def left_front_leg_step_forward(self,dev_angle:int=20,powerMode:bool=True):
        #dev_angle = 00;
        self.robot_stand();
        self.pass_servo_cmd(90-dev_angle,0,powerMode);   
        self.pass_servo_cmd(90+dev_angle,4,powerMode);
        self.left_front_leg_step_forward_state = self.get_state();
        
    def right_back_leg_step_forward(self,dev_angle:int=20,powerMode:bool=True):
        #dev_angle = 00;
        self.robot_stand();
        self.pass_servo_cmd(90+dev_angle,2,powerMode);   
        self.pass_servo_cmd(90-dev_angle,6,powerMode);
        self.right_back_leg_step_forward_state = self.get_state();
        
    ## AMBLE trait states
    def BL_pos(self,dev_angle:int=20,powerMode:bool=True):
        #self.robot_stand();
        self.pass_servo_cmd(90+dev_angle,1,powerMode);   
        self.pass_servo_cmd(90+dev_angle,5,powerMode);
        self.pass_servo_cmd(90+dev_angle,2,powerMode);   
        self.pass_servo_cmd(90+dev_angle,6,powerMode);
        self.pass_servo_cmd(90-dev_angle/3,3,powerMode);   
        self.pass_servo_cmd(90-dev_angle*1.5,7,powerMode);
        
        self.pass_servo_cmd(90+dev_angle/3,0,powerMode);   
        self.pass_servo_cmd(90-dev_angle*2.5,4,powerMode);
        self.BL_pos_state = self.get_state();
        
    def FL_pos(self,dev_angle:int=20,powerMode:bool=True):
        #self.robot_stand();
        self.pass_servo_cmd(90+dev_angle,1,powerMode);   
        self.pass_servo_cmd(90+dev_angle,5,powerMode);
        self.pass_servo_cmd(90+dev_angle,2,powerMode);   
        self.pass_servo_cmd(90+dev_angle,6,powerMode);
        self.pass_servo_cmd(90-dev_angle/3,3,powerMode);   
        self.pass_servo_cmd(90-dev_angle*1.5,7,powerMode);
        
        self.pass_servo_cmd(90-dev_angle/2,0,powerMode);   
        self.pass_servo_cmd(90-dev_angle*2.5,4,powerMode);
        self.FL_pos_state = self.get_state();   
        
    ## Oscillatory leg position functions
    def oscillate_leg_lft(self,leg_number:int=0,param_angle:float=0,powerMode:bool=True):
        pi = 22/7;
        
        step_forward_step_lft = np.array([100,180]);state_vect_sf_lft = np.array([1,0]);
        middle_state_lft = np.array([120,90]); state_vect_ms_lft = np.array([0,1]);
        step_backward_step_lft = np.array([140,180]);state_vect_sb_lft = np.array([-1,0]);
        middle_state_lft_neg = np.array([120,90]); state_vect_ms_lft_neg = np.array([0,-1]);
        
        param_vect = np.array([math.cos(param_angle),math.sin(param_angle)]);
        motor_states = (np.max([-0,np.dot(param_vect,state_vect_sf_lft)])*step_forward_step_lft + \
                       np.max([-0,np.dot(param_vect,state_vect_ms_lft)])*middle_state_lft + \
                       np.max([-0,np.dot(param_vect,state_vect_sb_lft)])*step_backward_step_lft) + \
                       np.max([-0,np.dot(param_vect,state_vect_ms_lft_neg)])*middle_state_lft_neg;
        motor_states = motor_states/( np.max([0,np.dot(param_vect,state_vect_sf_lft)]) + \
                                      np.max([0,np.dot(param_vect,state_vect_ms_lft)]) + \
                                      np.max([0,np.dot(param_vect,state_vect_sb_lft)]) + \
                                      np.max([0,np.dot(param_vect,state_vect_ms_lft_neg)]))
        if leg_number==0 or leg_number == 3:
            self.pass_servo_cmd(motor_states[0],leg_number,False);
            self.pass_servo_cmd(motor_states[1],leg_number+4,False);
        else:
            raise(ValueError);
        
        if(powerMode): # Sets all pwm signals to OFF to reduce load
            time.sleep(0.01);
            self.kit.servo[0].angle = None;
            self.kit.servo[1].angle = None;
            self.kit.servo[2].angle = None;
            self.kit.servo[3].angle = None;
            self.kit.servo[4].angle = None;
            self.kit.servo[5].angle = None;
            self.kit.servo[6].angle = None;
            self.kit.servo[7].angle = None;
        return motor_states
    
    def oscillate_leg_rght(self,leg_number:int=0,param_angle:float=0,powerMode:bool=True):
        pi = 22/7;
        
        step_forward_step_lft = np.array([80,0]);state_vect_sf_lft = np.array([-1,0]);
        middle_state_lft = np.array([60,90]); state_vect_ms_lft = np.array([0,1]);
        step_backward_step_lft = np.array([40,0]);state_vect_sb_lft = np.array([1,0]);
        middle_state_lft_neg = np.array([60,90]); state_vect_ms_lft_neg = np.array([0,-1]);
        
        param_vect = np.array([math.cos(param_angle),math.sin(param_angle)]);
        motor_states = (np.max([-0,np.dot(param_vect,state_vect_sf_lft)])*step_forward_step_lft + \
                       np.max([-0,np.dot(param_vect,state_vect_ms_lft)])*middle_state_lft + \
                       np.max([-0,np.dot(param_vect,state_vect_sb_lft)])*step_backward_step_lft) + \
                       np.max([-0,np.dot(param_vect,state_vect_ms_lft_neg)])*middle_state_lft_neg;
        motor_states = motor_states/( np.max([0,np.dot(param_vect,state_vect_sf_lft)]) + \
                                      np.max([0,np.dot(param_vect,state_vect_ms_lft)]) + \
                                      np.max([0,np.dot(param_vect,state_vect_sb_lft)]) + \
                                      np.max([0,np.dot(param_vect,state_vect_ms_lft_neg)]))
        if leg_number==1 or leg_number == 2:
            self.pass_servo_cmd(motor_states[0],leg_number,False);
            self.pass_servo_cmd(motor_states[1],leg_number+4,False);
        else:
            raise(ValueError);
        
        if(powerMode): # Sets all pwm signals to OFF to reduce load
            time.sleep(0.01);
            self.kit.servo[0].angle = None;
            self.kit.servo[1].angle = None;
            self.kit.servo[2].angle = None;
            self.kit.servo[3].angle = None;
            self.kit.servo[4].angle = None;
            self.kit.servo[5].angle = None;
            self.kit.servo[6].angle = None;
            self.kit.servo[7].angle = None;
        return motor_states
    
        
        
        
        
    def go_to_state(self,state_vector:list=[90,90,90,90,90,90,90,90],step:float=1,powermode:bool=True,leg_collision_thresh:float=5):
        # returns to the desired limb configs
        diff_vector = [self.servo0angle-state_vector[0], \
                       self.servo1angle-state_vector[1],\
                       self.servo2angle-state_vector[2],\
                       self.servo3angle-state_vector[3],\
                       self.servo4angle-state_vector[4],\
                       self.servo5angle-state_vector[5],\
                       self.servo6angle-state_vector[6],\
                       self.servo7angle-state_vector[7]];
        num_steps = max( [abs(diff_vector[0]),\
                          abs(diff_vector[1]),\
                          abs(diff_vector[2]),\
                          abs(diff_vector[3]),\
                          abs(diff_vector[4]),\
                          abs(diff_vector[5]),\
                          abs(diff_vector[6]),\
                          abs(diff_vector[7])] )/step;
        print(diff_vector)
        servo0angle = self.servo0angle;
        servo1angle = self.servo1angle;
        servo2angle = self.servo2angle;
        servo3angle = self.servo3angle;
        servo4angle = self.servo4angle;
        servo5angle = self.servo5angle;
        servo6angle = self.servo6angle;
        servo7angle = self.servo7angle;
        
        for i in range(0,int(num_steps)):
            #print(motor_top,motor_down,current_top + diff_vector[0]/abs(diff_vector[0]+0.01)*step,current_bottom + diff_vector[1]/abs(diff_vector[1]+0.01)*step);
            
            servo0angle = servo0angle - diff_vector[0]/abs(diff_vector[0]+0.01)*step;
            self.pass_servo_cmd(servo0angle,0,powermode);
            
            servo1angle = servo1angle - diff_vector[1]/abs(diff_vector[1]+0.01)*step;
            self.pass_servo_cmd(servo1angle,1,powermode);
            
            servo2angle = servo2angle - diff_vector[2]/abs(diff_vector[2]+0.01)*step;
            self.pass_servo_cmd(servo2angle,2,powermode);
            
            servo3angle = servo3angle - diff_vector[3]/abs(diff_vector[3]+0.01)*step;
            self.pass_servo_cmd(servo3angle,3,powermode);
            
            servo4angle = servo4angle - diff_vector[4]/abs(diff_vector[4]+0.01)*step;
            self.pass_servo_cmd(servo4angle,4,powermode);
            
            servo5angle = servo5angle - diff_vector[5]/abs(diff_vector[5]+0.01)*step;
            self.pass_servo_cmd(servo5angle,5,powermode);
            
            servo6angle = servo6angle - diff_vector[6]/abs(diff_vector[6]+0.01)*step;
            self.pass_servo_cmd(servo6angle,6,powermode);
            
            servo7angle = servo7angle - diff_vector[7]/abs(diff_vector[7]+0.01)*step;
            self.pass_servo_cmd(servo7angle,7,powermode);
            self.detect_leg_collisions(leg_collision_thresh);
                    
            diff_vector = [servo0angle-state_vector[0], \
                       servo1angle-state_vector[1],\
                       servo2angle-state_vector[2],\
                       servo3angle-state_vector[3],\
                       servo4angle-state_vector[4],\
                       servo5angle-state_vector[5],\
                       servo6angle-state_vector[6],\
                       servo7angle-state_vector[7]];
            self.servo0angle = servo0angle;
            self.servo1angle = servo1angle;
            self.servo2angle = servo2angle;
            self.servo3angle = servo3angle;
            self.servo4angle = servo4angle;
            self.servo5angle = servo5angle;
            self.servo6angle = servo6angle;
            self.servo7angle = servo7angle;
            
    def detect_leg_collisions(self,thresh:float=5.0):
        # Use it as a safety feature to avoid leg collisions
        left_legs_dis_x =  abs(self.calculate_legJoint_position(0)[0]-self.calculate_legJoint_position(3)[0]);
        right_legs_dis_x =  abs(self.calculate_legJoint_position(1)[0]-self.calculate_legJoint_position(2)[0]);
        if (min(left_legs_dis_x,right_legs_dis_x)<=thresh):
            print('-> PROXIMITY WARNING of LEGS',left_legs_dis_x,right_legs_dis_x);
            return True;
        else:
            print('-> safe leg positions',left_legs_dis_x,right_legs_dis_x);
            return False;
            raise(SystemError);

    def get_state(self):
        # returns the state of the robot servo motors in a list
        return [self.servo0angle,self.servo1angle, self.servo2angle, self.servo3angle,\
                self.servo4angle, self.servo5angle, self.servo6angle, self.servo7angle];
    
    def calculate_legJoint_position(self,leg:int=0):
        # Using simple inverse kinematics to track the limb positions
        # in real space. Used by the child class robot_state_display to
        # display limb positions real time
        pi = 22/7;
        if leg == 0:
            #top_motor_angle = (self.servo0angle*pi/180) - pi/2 + pi/6;
            top_motor_angle = (180-self.servo0angle - 30)*pi/180;
            joint_pos_x = -10 + 10*math.cos(top_motor_angle);
            joint_pos_y = 0 - 10*math.sin(top_motor_angle);
            
            bottom_motor_angle = top_motor_angle - (self.servo4angle)*pi/180;
            joint_pos_xb = joint_pos_x - 10*math.cos(bottom_motor_angle);
            joint_pos_yb = joint_pos_y + 10*math.sin(bottom_motor_angle);
        elif leg == 3:
            top_motor_angle = (180-self.servo3angle - 30)*pi/180;
            joint_pos_x = +10 + 10*math.cos(top_motor_angle);
            joint_pos_y = 0 - 10*math.sin(top_motor_angle);
            
            bottom_motor_angle = top_motor_angle - (self.servo7angle)*pi/180;
            joint_pos_xb = joint_pos_x - 10*math.cos(bottom_motor_angle);
            joint_pos_yb = joint_pos_y + 10*math.sin(bottom_motor_angle);
            
        elif leg == 1:
            top_motor_angle = ((self.servo1angle-30*1)*pi/180);
            joint_pos_x = -10 + 10*math.cos(top_motor_angle);
            joint_pos_y = 0 - 10*math.sin(top_motor_angle);
            
            bottom_motor_angle = pi - (top_motor_angle + (self.servo5angle)*pi/180);
            joint_pos_xb = joint_pos_x - 10*math.cos(bottom_motor_angle);
            joint_pos_yb = joint_pos_y - 10*math.sin(bottom_motor_angle);
            
        elif leg == 2:
            top_motor_angle = ((self.servo2angle-30*1)*pi/180);
            joint_pos_x = +10 + 10*math.cos(top_motor_angle);
            joint_pos_y = 0 - 10*math.sin(top_motor_angle);
            
            bottom_motor_angle = pi - (top_motor_angle + (self.servo6angle)*pi/180);
            joint_pos_xb = joint_pos_x - 10*math.cos(bottom_motor_angle);
            joint_pos_yb = joint_pos_y - 10*math.sin(bottom_motor_angle);
        else:
            print('Wrong leg number. Please enter from 0-3');
        print('-> Leg joint of leg', leg, 'POSITION [X,Y] =',joint_pos_x,joint_pos_y);    
        return [joint_pos_x,joint_pos_y,joint_pos_xb,joint_pos_yb]
    
    
    
        
    
    
    
        
        
    
    
if __name__ == "__main__":
    # Initi
    s = quadropodMK4(0x40,False);
    s.robot_stand(40,False)
    s.robot_sit(40,False)
    s.left_front_leg_step_forward();
    s.right_back_leg_step_forward();
    s.BL_pos();
    s.FL_pos();
    iter = 0;
    param_angle1 = 0;
    param_angle2 = 0;
    pi = 22/7;
    while(iter<1000):
        s.oscillate_leg_lft(0,1*param_angle1 +2*1*2*pi/6,False)
        s.oscillate_leg_rght(1,1*param_angle1 + 1*2*pi/6,False)
        s.oscillate_leg_rght(2,1*param_angle2 + 2*1*2*pi/6,False)        
        s.oscillate_leg_lft(3,1*param_angle2 + 1*2*pi/6,False)
        
        param_angle1 -= 0.1;
        param_angle2 += 0.1;
        #s.go_to_state(s.left_front_leg_step_forward_state,5,False,5)
        time.sleep(.1);
        #s.go_to_state(s.right_back_leg_step_forward_state,5,False,5)
        #time.sleep(1);
        #s.robot_sit(40,True)
        #time.sleep(1);
        iter += 1;
        
        
s.pass_servo_cmd(90,0)
s.pass_servo_cmd(90,1)
s.pass_servo_cmd(90,2)
s.pass_servo_cmd(90,3)
s.pass_servo_cmd(90,4)
s.pass_servo_cmd(90,5)
s.pass_servo_cmd(90,6)
s.pass_servo_cmd(90,7)