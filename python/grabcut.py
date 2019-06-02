import time
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

road = cv.imread('/home/ben/apr13/test-1555162200/color/000000.jpg')

height , width , layers =  road.shape
video = cv.VideoWriter('just_road.avi', cv.VideoWriter_fourcc(*"MJPG"), 30, (width,height))

for frame_idx in range(1500):
    #print("{}".format(frame_idx))
    idx = '{0:06d}'.format(frame_idx)
    mask = cv.imread('/home/ben/apr13/test-1555162200/masks/'+idx+'_pred_labelIds.png')
    mask = cv.cvtColor(mask,cv.COLOR_BGR2GRAY)
    road = cv.imread('/home/ben/apr13/test-1555162200/color/'+idx+'.jpg')

    #Downsample for faster grabcut
    #mask_down = cv.pyrDown(mask)
    #road_down = cv.pyrDown(road)
    mask_down = mask.copy()
    road_down = road.copy()

    kernel = np.ones((3,3),np.uint8)
    erosion = cv.erode(mask_down, kernel, iterations = 25)

    gcmask = cv.addWeighted(erosion, 0.5, mask_down, 0.5,0)
    gcmask = np.where((gcmask != 128)&(gcmask != 255), cv.GC_BGD, gcmask)
    gcmask[gcmask == 128] = cv.GC_PR_FGD
    gcmask[gcmask == 255] = cv.GC_FGD

    backgroundModel = np.zeros((1, 65), np.float64)
    foregroundModel = np.zeros((1, 65), np.float64)

    rectangle = (150,150,100,100)

    start = time.time()

    cv.grabCut(road_down, gcmask, rectangle, 
                backgroundModel, foregroundModel,
                1, cv.GC_INIT_WITH_MASK)

    print(time.time() - start)

    mask2 = np.where((mask_down == 2)|(mask_down == 0), 0, 1).astype('uint8')
    mask2 = cv.pyrUp(mask2)

    gcroad = road * mask2[:, :, np.newaxis]

    # video.write(road)
    # output segmented image with colorbar
    cv.imshow('road', gcroad)
    cv.waitKey(10)

#cv.destroyAllWindows()
#video.release()
