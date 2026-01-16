import pygame
import os
import UWB_Reader
import numpy as np
import time

pygame.font.init()

#MOVING AVERAGE FILTER WINDOW SIZE
MAF_WIN_X = 10
MAF_WIN_Y = 10

# The drones data
SPACESHIP_WIDTH = 20
SPACESHIP_HEIGHT = 20

# The window's width and height in pixel
WIDTH, HEIGHT = 494, 500

#Field WIDTH and HEIGHT
FIELD_WIDTH, FIELD_HEIGHT = 19.8, 19.8 #In meters

# Change the scale of the window and display it
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Field Moving Screen")

# The fps, frames per second
FPS = 60

# Load the picture 
SPACESHIP_PIC = pygame.image.load(
    os.path.join('resources','spaceship.png'))
# Load the background and change its scale
FIELD = pygame.transform.scale(
pygame.image.load(os.path.join('resources', 'field2025_old_overlay.png')), (494,500)) #resources\field2025_old_overlay.png

# Rotate the spaceship and its scale
SPACESHIP = pygame.transform.rotate(pygame.transform.scale(
    SPACESHIP_PIC, (SPACESHIP_WIDTH,SPACESHIP_HEIGHT)), 0)

class MAF:
    def __init__(self, window):
        self.winsize = window
        self.data = np.array([])
        self.average = []
    
    def process(self, value):
        self.data = np.append(self.data, value)
        meanValue = np.mean(self.data)
        self.average.append(meanValue)
        if len(self.data) == self.winsize:
           self.data = np.delete(self.data, 0)
    
    def getAverage(self):
        return self.average
        
# Draw the window 
def draw_window(space_ship):
    WIN.blit(FIELD,(0,0))
    WIN.blit(SPACESHIP,(space_ship.x,space_ship.y))
    pygame.display.update() 
    
def main():
    pose2d = [0,0]
    '''
    x0 = 1.14
    y0 = 0.82
    pose = (int(x0/FIELD_WIDTH*494),int(y0/FIELD_HEIGHT*500),0) #Original position
    spaceship = pygame.Rect(pose[0],pose[1],SPACESHIP_WIDTH,SPACESHIP_HEIGHT)
    '''
    raw_path = []
    filter_path = []
    clock = pygame.time.Clock()
    
    run = True

    mafx = MAF(MAF_WIN_X)
    mafy = MAF(MAF_WIN_Y)

    i = 0
    WIN.blit(FIELD,(0,0))
    #start_time = time.time()
    while run:
        clock.tick(FPS) 
        pose = UWB_Reader.get_target_position(8)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit() 
        raw_path.append((pose[0], pose[1]))
        mafx.process(pose[0])
        mafy.process(pose[1])
        
        x_filt = mafx.getAverage()[i]
        y_filt = mafy.getAverage()[i]
        filter_path.append((x_filt, y_filt))
        pose2d[0] = int(x_filt/FIELD_WIDTH*494)
        pose2d[1] = int(y_filt/FIELD_HEIGHT*500)
        print("The position of the pose", pose2d)

        #Uncommand this line to display everything in dot
        #pygame.draw.circle(WIN, (255,0,0), (pose2d[0], pose2d[1]), 2)
        
        if len(raw_path) > 1:
            pygame.draw.lines(WIN, (255,0,255), False, [(int(y/FIELD_HEIGHT*494), int(x/FIELD_WIDTH*500)) for (x, y) in raw_path], 3)
        # draw recorded filtered path
        if len(filter_path) > 1:
            pygame.draw.lines(WIN, (0, 255, 0), False, [(int(y/FIELD_HEIGHT*494), int(x/FIELD_WIDTH*500)) for (x, y) in filter_path], 3)
        pygame.display.update() 
        i += 1

     
if __name__ == "__main__": #only run in file name main 
    main()