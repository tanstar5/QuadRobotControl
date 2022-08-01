import pygame
import numpy as np
#from quadropodMK3 import quadropodMK3

class robot_state_display(quadropodMK3):    
    def __init__(self,state_vector:'list'=[90,90,90,90,90,90,90,90],\
                 WIDTH:'int'=900,HEIGHT:'int'=500):
        self.WIDTH = WIDTH;
        self.HEIGHT = HEIGHT;
        self.state_vector = state_vector;
        
    def draw_robot_state(self,robot_obj, scale:'int'=6):
        self.WIN.fill((0,0,100));
        coors = [];
        #Draw the backbone
        origin = (int(self.WIDTH/2),int(self.HEIGHT/2));
        pygame.draw.circle(self.WIN,(255,0,0),(origin[0]-10*scale,origin[1]-10*scale),20);        
        pygame.draw.circle(self.WIN,(255,0,0),(origin[0]+10*scale,origin[1]-10*scale),15);
        pygame.draw.line(self.WIN,(0,255,0),(origin[0]-10*scale,origin[1]-10*scale),(origin[0]+10*scale,origin[1]-10*scale),5);
        #Draw legs________________________________
        leg0_x = origin[0] + int(robot_obj.calculate_legJoint_position(0)[0])*scale;
        leg0_y = origin[1] - int(robot_obj.calculate_legJoint_position(0)[1])*scale;
        pygame.draw.circle(self.WIN,(255,0,100),(leg0_x,leg0_y),15);
        pygame.draw.line(self.WIN,(0,255,0),(origin[0]-10*scale,origin[1]-10*scale),(leg0_x,leg0_y),2);
        
        leg3_x = origin[0] + int(robot_obj.calculate_legJoint_position(3)[0])*scale;
        leg3_y = origin[1] - int(robot_obj.calculate_legJoint_position(3)[1])*scale;
        pygame.draw.circle(self.WIN,(255,0,100),(leg3_x,leg3_y),15);
        pygame.draw.line(self.WIN,(0,255,0),(origin[0]+10*scale,origin[1]-10*scale),(leg3_x,leg3_y),2);
        
        leg1_x = origin[0] + int(robot_obj.calculate_legJoint_position(1)[0])*scale;
        leg1_y = origin[1] - int(robot_obj.calculate_legJoint_position(1)[1])*scale;
        pygame.draw.circle(self.WIN,(255,0,100),(leg1_x,leg1_y),15);
        
        leg2_x = origin[0] + int(robot_obj.calculate_legJoint_position(2)[0])*scale;
        leg2_y = origin[1] - int(robot_obj.calculate_legJoint_position(2)[1])*scale;
        pygame.draw.circle(self.WIN,(255,0,100),(leg2_x,leg2_y),15);    
        pygame.display.update();
    
    def visualize(self,robot_obj,bg_color:'tuple'=(0,0,30)):
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