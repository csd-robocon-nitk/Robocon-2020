import pygame
import numpy as np
import sys
import cv2 as cv
from freenect import sync_get_depth as get_depth


def make_gamma():
    """
    Create a gamma table
    """
    num_pix = 2048 # there's 2048 different possible depth values
    npf = float(num_pix)
    _gamma = np.empty((num_pix, 3), dtype=np.uint16)
    for i in xrange(num_pix):
        v = i / npf
        v = pow(v, 3) * 6
        pval = int(v * 6 * 256)
        lb = pval & 0xff
        pval >>= 8
        if pval == 0:
            a = np.array([255, 255 - lb, 255 - lb], dtype=np.uint8)
        elif pval == 1:
            a = np.array([255, lb, 0], dtype=np.uint8)
        elif pval == 2:
            a = np.array([255 - lb, lb, 0], dtype=np.uint8)
        elif pval == 3:
            a = np.array([255 - lb, 255, 0], dtype=np.uint8)
        elif pval == 4:
            a = np.array([0, 255 - lb, 255], dtype=np.uint8)
        elif pval == 5:
            a = np.array([0, 0, 255 - lb], dtype=np.uint8)
        else:
            a = np.array([0, 0, 0], dtype=np.uint8)

        _gamma[i] = a
    return _gamma


gamma = make_gamma()


if __name__ == "__main__":
    fpsClock = pygame.time.Clock()
    FPS = 30 # kinect only outputs 30 fps
    disp_size = (640, 480)
    pygame.init()
    screen = pygame.display.set_mode(disp_size)
    font = pygame.font.SysFont('comicsans', 32) # provide your own font
    #np.set_printoptions(threshold=sys.maxsize)
    while True:
        events = pygame.event.get()
        for e in events:
            if e.type == pygame.QUIT:
                sys.exit()
        fps_text = "FPS: {0:.2f}".format(fpsClock.get_fps())
        # draw the pixels

        depth = np.rot90(get_depth()[0]) # get the depth readinngs from the camera

        pixels = gamma[depth] # the colour pixels are the depth readings overlayed onto the gamma table
	depth = 100.0 / (depth*-0.0030711016 + 3.3309495161);
	depth = depth-(0.0003*(depth)*(depth)-0.09633*(depth)-0.6032)- 0.04*(depth) +0.511;
#	sum = 0;
#	im2, contours, hierarchy = cv.findContours(depth, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#	cv.drawContours(img, contours, -1, (0,255,0), 3)

	#for i in range(640):
	#	if(depth[i][0] < 0):
	#		sum = sum + depth[i-100][j]
	#
#
#	print(sum/240)
    for i in range(640):
        if(depth[i][240]-depth[i+1]>50):
            b1=i
        elif(depth[i][240]-depth[i+1]<50):
            b2=i
    b3=(b1+b2)/2
	print(depth[320][240])
	#print(depth.shape)
	#print(pixels)
	temp_surface = pygame.Surface(disp_size)
        pygame.surfarray.blit_array(temp_surface, pixels)
        pygame.transform.scale(temp_surface, disp_size, screen)
        screen.blit(font.render(fps_text, 1, (255, 255, 255)), (30, 30))
        pygame.display.flip()
        fpsClock.tick(FPS)
