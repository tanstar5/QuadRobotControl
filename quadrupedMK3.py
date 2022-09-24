# imports for this class_______________
#import numpy as np #A neede module which will be used  in later release 
import time
import pygame
import math

#from robot_state_display import robot_state_display


# Important servo controll classes from adafruit libraries
from adafruit_servokit import ServoKit;
import adafruit_motor.servo;

class quadropodMK3:
    # Control object for servo motors of the quadrupedMK3 robot______________
    # Please ensure to setup up circuitpython kit from adafruit for controlling Servo motors
    # Then please ensure arrangement of the servo motor address (For this project top motors are numbered in clockwise
    #      direction starting from left front limb followed by the bottom motors)
    # Then please set up the degree of freedom for the servo motors to avoid leg collisions by setting up the min and max
    #      range values in the __init__ routine
    # Then pass all the motor commands using pass_servo_cmd
    # Developed by Tanumoy Saha; push/pull: @ tanumoysaha5@gmail.com.
    
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
            self.servo0_min = 0;self.servo0_max = 120;
            self.servo4_min = 0;self.servo4_max = 180;
            
            self.servo1_min = 60;self.servo1_max = 180;
            self.servo5_min = 0;self.servo5_max = 180;
            
            self.servo2_min = 60;self.servo2_max = 120;
            self.servo6_min = 0;self.servo6_max = 180;
            
            self.servo3_min = 0;self.servo3_max = 120;
            self.servo7_min = 0;self.servo7_max = 180;
            
            # Robot state vectors
            self.state_vector_leanBackward = [120, 60, 60, 120, 90, 90, 120, 60];
            self.state_vector_leanForward = [120, 60, 60, 120, 60, 120, 90, 90];
            self.state_vector_leanLeft = [110, 90, 90, 110, 70, 70, 70, 70];
            self.state_vector_leanRight = [90, 70, 70, 90, 110, 110, 110, 110]
            
            # Robots walking state vectors
            self.state_vector_walk_init = [90, 90, 90, 90, 120, 60, 60, 120];
            
            # Robot's desired state vectors
            if instant_control:
                pygame.init()
                win = pygame.display.set_mode((100,100));
            print('->_init_: The robot is initialized \n');
            
            #Robots state display tv
            self.tv = robot_state_display();
    #__________________________________________________________________________________
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
            
    
    def switchoff_allPWM(self):
        self.kit.servo[0].angle = None;
        self.kit.servo[1].angle = None;
        self.kit.servo[2].angle = None;
        self.kit.servo[3].angle = None;
        self.kit.servo[4].angle = None;
        self.kit.servo[5].angle = None;
        self.kit.servo[6].angle = None;
        self.kit.servo[7].angle = None;
    
    
    # Robot default position modes________________________________________________        
    def robot_stand(self,powerMode:bool=True):
        dev_angle = 10;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90,4,powerMode);
        self.pass_servo_cmd(90,5,powerMode);
        self.pass_servo_cmd(90,6,powerMode);
        self.pass_servo_cmd(90,7,powerMode);
        
        
        
    def robot_sit(self,powerMode:bool=True):
        dev_angle = 30;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90-dev_angle,4,powerMode);
        self.pass_servo_cmd(90+dev_angle,5,powerMode);
        self.pass_servo_cmd(90+dev_angle,6,powerMode);
        self.pass_servo_cmd(90-dev_angle,7,powerMode);
        
    def robot_rest(self,powerMode:bool=True):
        dev_angle = 30;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90-1.5*dev_angle,4,powerMode);
        self.pass_servo_cmd(90+1.5*dev_angle,5,powerMode);
        self.pass_servo_cmd(90+1.5*dev_angle,6,powerMode);
        self.pass_servo_cmd(90-1.5*dev_angle,7,powerMode);    
    
    def robot_lean_backward(self,powerMode:bool=True):
        dev_angle = 30;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90,4,powerMode);
        self.pass_servo_cmd(90,5,powerMode);
        self.pass_servo_cmd(90+dev_angle,6,powerMode);
        self.pass_servo_cmd(90-dev_angle,7,powerMode);
        self.state_vector_leanBackward = [120, 60, 60, 120, 90, 90, 120, 60] #[self.servo0angle, self.servo1angle, self.servo2angle, self.servo3angle, \
                                                       #self.servo4angle, self.servo5angle, self.servo6angle, self.servo7angle,];
        
    def robot_walk_mode(self,powerMode:bool=True):
        dev_angle = 30;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90,4,powerMode);
        self.pass_servo_cmd(90,5,powerMode);
        self.pass_servo_cmd(90+.4*dev_angle,6,powerMode);
        self.pass_servo_cmd(90-.4*dev_angle,7,powerMode);    
        
    def robot_lean_forward(self,powerMode:bool=True):
        dev_angle = 30;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90-dev_angle,4,powerMode);
        self.pass_servo_cmd(90+dev_angle,5,powerMode);
        self.pass_servo_cmd(90,6,powerMode);
        self.pass_servo_cmd(90,7,powerMode);
        self.state_vector_leanForward = [120, 60, 60, 120, 60, 120, 90, 90]
        
    def robot_lean_right(self,powerMode:bool=True):
        dev_angle = 20;
        self.pass_servo_cmd(90,0,powerMode);
        self.pass_servo_cmd(90-dev_angle,1,powerMode);
        self.pass_servo_cmd(90-dev_angle,2,powerMode);
        self.pass_servo_cmd(90,3,powerMode);
        
        self.pass_servo_cmd(90+dev_angle,4,powerMode);
        self.pass_servo_cmd(90+dev_angle,5,powerMode);
        self.pass_servo_cmd(90+dev_angle,6,powerMode);
        self.pass_servo_cmd(90+dev_angle,7,powerMode);
        
        self.state_vector_leanRight = [90, 70, 70, 90, 110, 110, 110, 110]
    
    def robot_lean_left(self,powerMode:bool=True):
        dev_angle = 20;
        self.pass_servo_cmd(90+dev_angle,0,powerMode);
        self.pass_servo_cmd(90,1,powerMode);
        self.pass_servo_cmd(90,2,powerMode);
        self.pass_servo_cmd(90+dev_angle,3,powerMode);
        
        self.pass_servo_cmd(90-dev_angle,4,powerMode);
        self.pass_servo_cmd(90-dev_angle,5,powerMode);
        self.pass_servo_cmd(90-dev_angle,6,powerMode);
        self.pass_servo_cmd(90-dev_angle,7,powerMode);
        
        self.state_vector_leanLeft = [110, 90, 90, 110, 70, 70, 70, 70];
    
    # Define safety functions_____________________________________
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
        
        
    # Walking functions ________________________________________________________________________    
    def get_state(self):
        # returns the state of the robot servo motors in a list
        return [self.servo0angle,self.servo1angle, self.servo2angle, self.servo3angle,\
                self.servo4angle, self.servo5angle, self.servo6angle, self.servo7angle,];
        
    def go_to_position_leg(self,leg_no:int=0,pos:list=[90,90],step:float=1,powermode:bool=True):
        # conforms to the desired leg state
        if leg_no == 0:
            current_top = self.servo0angle;
            current_bottom = self.servo4angle;
            motor_top = 0;
            motor_down = 4;
        elif leg_no == 1:
            current_top = self.servo1angle;
            current_bottom = self.servo5angle;
            motor_top = 1;
            motor_down = 5;
        elif leg_no == 2:
            current_top = self.servo2angle;
            current_bottom = self.servo6angle;
            motor_top = 2;
            motor_down = 6;
        elif leg_no == 3:
            current_top = self.servo3angle;
            current_bottom = self.servo7angle;
            motor_top = 3;
            motor_down = 7;
        else:
            print('-> WRONG leg number');
            raise(ValueError);
        
        diff_vector = [current_top-pos[0],current_bottom-pos[1] ];
        print(diff_vector)
        num_steps = max( [abs(diff_vector[0]),abs(diff_vector[1])] )/step;
        
        for i in range(0,int(num_steps)):
            print(motor_top,motor_down,current_top + diff_vector[0]/abs(diff_vector[0]+0.01)*step,current_bottom + diff_vector[1]/abs(diff_vector[1]+0.01)*step);
            current_top = current_top - diff_vector[0]/abs(diff_vector[0]+0.01)*step;
            self.pass_servo_cmd(current_top,motor_top,powermode);
            
            current_bottom = current_bottom - diff_vector[1]/abs(diff_vector[1]+0.01)*step;
            self.pass_servo_cmd(current_bottom,motor_down,powermode);        
            diff_vector = [current_top-pos[0],current_bottom-pos[1] ];
        

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
            self.tv.draw_robot_state(self)
            
    def joystick_robot_poses(self):
        pass
    
    
    def walk_stable1(self):
        # sequence of cmds for a stable walking gait
        s.go_to_state([100, 80, 80, 100, 90, 90, 90, 90],step,False,5)
        s.go_to_state([100, 80, 80, 100, 80, 90, 90, 90],step,False,5)
        s.go_to_state([60, 80, 80, 100, 120, 90, 90, 90],step,False,5)

        #move right front leg
        s.go_to_state([60, 80, 80, 100, 120, 90, 90, 90],step,False,5)
        s.go_to_state([60, 80, 80, 100, 120, 110, 90, 90],step,False,5)
        s.go_to_state([60, 120, 80, 100, 120, 60, 90, 90],step,False,5)

        #move left back leg
        s.go_to_state([60, 120, 80, 100, 120, 60, 90, 90],step,False,5)
        s.go_to_state([60, 120, 80, 100, 120, 60, 90, 80],step,False,5)
        s.go_to_state([60, 120, 80, 60, 120, 60, 90,  80],step,False,5)

        #move right back leg
        s.go_to_state([60, 120, 80, 60, 120, 60, 90, 80],step,False,5)
        s.go_to_state([60, 120, 80, 60, 120, 60, 110, 80],step,False,5)
        s.go_to_state([60, 120, 120, 60, 120, 60, 100, 80],step,False,5)
        
        
class robot_state_display(quadropodMK3):
    # child of the quadropodMK3 for displaying the limb configs in a pygame window
    def __init__(self,state_vector:'list'=[90,90,90,90,90,90,90,90],\
                 WIDTH:'int'=900,HEIGHT:'int'=500):
        self.WIDTH = WIDTH;
        self.HEIGHT = HEIGHT;
        self.state_vector = state_vector;
        self.WIN = pygame.display.set_mode( (self.WIDTH,self.HEIGHT) );
        pygame.display.set_caption("Robot state");
        
    def draw_robot_state(self,robot_obj, scale:'int'=10):
        self.WIN.fill((0,0,100));
        coors = [];
        #Draw the backbone
        origin = (int(self.WIDTH/2),int(self.HEIGHT/2));
        pygame.draw.circle(self.WIN,(255,0,0),(origin[0]-10*scale,origin[1]-0*scale),20);        
        pygame.draw.circle(self.WIN,(255,0,0),(origin[0]+10*scale,origin[1]-00*scale),15);
        pygame.draw.line(self.WIN,(0,255,0),(origin[0]-10*scale,origin[1]-0*scale),(origin[0]+10*scale,origin[1]-0*scale),5);
        #Draw legs________________________________
        leg0_x = origin[0] + math.floor(robot_obj.calculate_legJoint_position(0)[0])*scale;
        leg0_y = origin[1] - math.floor(robot_obj.calculate_legJoint_position(0)[1])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg0_x,leg0_y),15);
        pygame.draw.line(self.WIN,(0,255,0),(origin[0]-10*scale,origin[1]-0*scale),(leg0_x,leg0_y),2);
        
        leg0_xb = origin[0] + math.floor(robot_obj.calculate_legJoint_position(0)[2])*scale;
        leg0_yb = origin[1] - math.floor(robot_obj.calculate_legJoint_position(0)[3])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg0_xb,leg0_yb),15);
        pygame.draw.line(self.WIN,(0,255,0),(leg0_x,leg0_y),(leg0_xb,leg0_yb),2);
        
        leg3_x = origin[0] + math.floor(robot_obj.calculate_legJoint_position(3)[0])*scale;
        leg3_y = origin[1] - math.floor(robot_obj.calculate_legJoint_position(3)[1])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg3_x,leg3_y),15);
        pygame.draw.line(self.WIN,(0,255,0),(origin[0]+10*scale,origin[1]-0*scale),(leg3_x,leg3_y),2);
        
        leg3_xb = origin[0] + math.floor(robot_obj.calculate_legJoint_position(3)[2])*scale;
        leg3_yb = origin[1] - math.floor(robot_obj.calculate_legJoint_position(3)[3])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg3_xb,leg3_yb),15);
        pygame.draw.line(self.WIN,(0,255,0),(leg3_x,leg3_y),(leg3_xb,leg3_yb),2);
        
        leg1_x = origin[0] + math.floor(robot_obj.calculate_legJoint_position(1)[0])*scale;
        leg1_y = origin[1] - math.floor(robot_obj.calculate_legJoint_position(1)[1])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg1_x,leg1_y),15);
        pygame.draw.line(self.WIN,(0,128,0),(origin[0]-10*scale,origin[1]-0*scale),(leg1_x,leg1_y),2);
        
        leg1_xb = origin[0] + math.floor(robot_obj.calculate_legJoint_position(1)[2])*scale;
        leg1_yb = origin[1] - math.floor(robot_obj.calculate_legJoint_position(1)[3])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg1_xb,leg1_yb),15);
        pygame.draw.line(self.WIN,(0,128,0),(leg1_x,leg1_y),(leg1_xb,leg1_yb),2);
        
        leg2_x = origin[0] + math.floor(robot_obj.calculate_legJoint_position(2)[0])*scale;
        leg2_y = origin[1] - math.floor(robot_obj.calculate_legJoint_position(2)[1])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg2_x,leg2_y),15);
        pygame.draw.line(self.WIN,(0,128,0),(origin[0]+10*scale,origin[1]-0*scale),(leg2_x,leg2_y),2);
        
        leg2_xb = origin[0] + math.floor(robot_obj.calculate_legJoint_position(2)[2])*scale;
        leg2_yb = origin[1] - math.floor(robot_obj.calculate_legJoint_position(2)[3])*scale;
        pygame.draw.circle(self.WIN,(0,10,250),(leg2_xb,leg2_yb),15);
        pygame.draw.line(self.WIN,(0,128,0),(leg2_x,leg2_y),(leg2_xb,leg2_yb),2);
        pygame.display.update();
    
    def visualize(self,bg_color:'tuple'=(0,0,30)):
        clock = pygame.time.Clock();
        self.WIN = pygame.display.set_mode( (self.WIDTH,self.HEIGHT) );
        pygame.display.set_caption("Robot state");
        run = True;
#         while(run):
#             clock.tick(FPS);
#             for event in pygame.event.get():
#                 if event.type==pygame.QUIT:
#                     run = False;
#                 
#             self.WIN.fill(bg_color);
#             self.draw_robot_state();
#             pygame.display.update();
#             #_Write the code here---__________________
#             self.draw_robot_state(robot_obj);
            
            #_________________________________________
        #pygame.quit();        
        
        

if __name__ == "__main__":
    s = quadropodMK3(0x40,False);
    tv = robot_state_display();
    tv.visualize((0,0,30));
    iter = 0;
    step = 5;
    powermode = False;
    leg_collision_thresh = 5;

    s.robot_rest(False);
    

    # s.go_to_state([90.0, 90.0, 90.0, 90.0, 120, 60, 60, 120],1,False,5);
    time.sleep(1);
    # s.go_to_state([60.0, 90.0, 90.0, 90.0, 150, 60, 60, 120],1,False,5);
    # time.sleep(1);
    # s.go_to_state([60.0, 90.0, 120.0, 90.0, 150, 60, 60, 120],1,False,5);
    # time.sleep(1);
    # s.go_to_state([60.0, 90.0, 120.0, 60.0, 150, 60, 60, 120],1,False,5);
    # time.sleep(1);
    # s.go_to_state([60.0, 120.0, 120.0, 60.0, 150, 30, 60, 120],1,False,5);
    # time.sleep(1);
    # s.go_to_state([90.0, 90.0, 90.0, 90.0, 150, 60, 60, 120],1,False,5);
    # # s.robot_walk_mode();

    step = 10
    #s.robot_rest(False);
    # time.sleep(1);
    # s.robot_sit();
    # time.sleep(1);
    # s.robot_lean_backward();
    # time.sleep(1);
    # s.robot_walk_mode();
    time.sleep(10);


    while(iter<1000):
        #s.walk_stable1();
        s.go_to_state([120, 60, 60, 120, 45.0, 135.0, 135.0, 45.0],step,False,5)   #rest
        
        #tv.draw_robot_state();
        time.sleep(1);
        s.go_to_state(s.state_vector_leanBackward,step,False,5)   #lean backward
        
        #tv.draw_robot_state();
        time.sleep(1);
        s.go_to_state(s.state_vector_leanForward,0.5*step,False,5)   #lean backward
        #tv.draw_robot_state();
        time.sleep(1);
        s.go_to_state(s.state_vector_leanBackward,step,False,5)   #lean backward
        #tv.draw_robot_state();
        time.sleep(1);
        s.go_to_state(s.state_vector_leanForward,0.5*step,False,5)   #lean backward
        #tv.draw_robot_state();
        time.sleep(1);
        
        
        s.go_to_state([100, 80, 80, 100, 90, 90, 90, 90],step,False,5)   #stand    
        time.sleep(1);
        
        s.go_to_state([110, 90, 90, 110, 70, 70, 70, 70],0.5*step,False,5)   #lean left
        time.sleep(1);
        s.go_to_state([100, 80, 80, 100, 90, 90, 90, 90],step,False,5)   #stand    
        time.sleep(1);
        s.go_to_state([90, 70, 70, 90, 110, 110, 110, 110],0.5*step,False,5)   #lean right
        time.sleep(1);
        s.go_to_state([100, 80, 80, 100, 90, 90, 90, 90],step,False,5)   #stand    
        time.sleep(1);
        
        time.sleep(2);
        iter +=1;


    # #move left front leg
    # s.go_to_state([100, 80, 80, 100, 90, 90, 90, 90],step,False,5)
    # s.go_to_state([100, 80, 80, 100, 80, 90, 90, 90],step,False,5)
    # s.go_to_state([60, 80, 80, 100, 120, 90, 90, 90],step,False,5)
    # 
    # #move right front leg
    # s.go_to_state([60, 80, 80, 100, 120, 90, 90, 90],step,False,5)
    # s.go_to_state([60, 80, 80, 100, 120, 110, 90, 90],step,False,5)
    # s.go_to_state([60, 120, 80, 100, 120, 60, 90, 90],step,False,5)
    # 
    # #move left back leg
    # s.go_to_state([60, 120, 80, 100, 120, 60, 90, 90],step,False,5)
    # s.go_to_state([60, 120, 80, 100, 120, 60, 90, 80],step,False,5)
    # s.go_to_state([60, 120, 80, 60, 120, 60, 90, 110],step,False,5)
    # 
    # #move right back leg
    # s.go_to_state([60, 120, 80, 60, 120, 60, 90, 110],step,False,5)
    # s.go_to_state([60, 120, 80, 60, 120, 60, 100, 110],step,False,5)
    # s.go_to_state([60, 120, 120, 60, 120, 60, 70, 110],step,False,5)

    # #Walk1_____________________________________________________________
    # s.go_to_state([90.0, 90.0, 90.0, 90.0, 120, 60, 60, 120],4,False,5);
    # time.sleep(.2);
    # s.go_to_state([60.0, 90.0, 90.0, 90.0, 150, 60, 60, 120],4,False,5);
    # time.sleep(.2);
    # s.go_to_state([60.0, 90.0, 120.0, 90.0, 150, 60, 60, 120],4,False,5);
    # time.sleep(.2);
    # s.go_to_state([60.0, 90.0, 120.0, 60.0, 150, 60, 60, 120],4,False,5);
    # time.sleep(.2);
    # s.go_to_state([60.0, 120.0, 120.0, 60.0, 150, 30, 60, 120],4,False,5);
    # time.sleep(.2);
    # 
    # s.go_to_state([90.0, 90.0, 90.0, 90.0, 150, 60, 60, 120],4,False,5);
    # time.sleep(.2);
    # s.go_to_state([90.0, 120.0, 90.0, 90.0, 120, 30, 60, 120],4,False,5);
    # time.sleep(.2);
    # s.go_to_state([90.0, 120.0, 90.0, 60.0, 120, 30, 60, 120],4,False,5);
    # time.sleep(.2);
# s.go_to_state([90.0, 120.0, 120.0, 60.0, 120, 30, 60, 120],4,False,5);
# time.sleep(.2);
# s.go_to_state([90.0, 120.0, 120.0, 60.0, 120, 30, 60, 120],4,False,5);
# time.sleep(.2);    
#s.robot_walk_mode()
