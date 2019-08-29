import cv2
import numpy as np
import imutils

def convert2list(img):
    height, width = img.shape
    maze = np.zeros((height,width), np.uint8)
    for i in range(width):
        for j in range(height): 
            # print(i, j)
            maze[j][i] = 1 if img[j][i] > 0 else 0

    return maze

def img2binList(lenWidth, GRID_SIZE=50, verbose=0):
    
    img = cv2.imread("test.jpg")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    _, gray = cv2.threshold(gray, 112, 255, cv2.THRESH_BINARY_INV)
    if verbose:
        cv2.imshow("img", gray)
        cv2.waitKey(0)
    
    
    cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    locs = []
    
    height, width = gray.shape
    tmp = np.zeros((height,width), np.uint8)
    
    idxLargest = 0
    areaLargest = 0
    # loop over the contours
    for (i, c) in enumerate(cnts):
        # compute the bounding box of the contour, then use the
        # bounding box coordinates to derive the aspect ratio
        (x, y, w, h) = cv2.boundingRect(c)
        if w*h > areaLargest:
            idxLargest = i
            areaLargest = w*h
        cv2.rectangle(tmp, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if verbose:
        # print("found largest contour outline")
        cv2.imshow("img", tmp)
        cv2.waitKey(0)
    
    # print("cropping image as largest contour")
    (x, y, w, h) = cv2.boundingRect(cnts[idxLargest])
    gray = gray[y:y+h, x:x+w]
    if verbose:
        cv2.imshow("img", gray)
        cv2.waitKey(0)

    print("the cropped maze size is "+str(w)+" X "+str(h))

    mapWidth = (int) (lenWidth // GRID_SIZE)
    mapHeight = (int) ((h/w)*lenWidth // GRID_SIZE)
    print("the map will be created by the size: "+str(mapWidth)+" X "+str(mapHeight))
    
    resized_gray = imutils.resize(gray, width=mapWidth) # resize the map for convolution
    _, resized_gray = cv2.threshold(resized_gray, 1, 255, cv2.THRESH_BINARY)
    if verbose:
        cv2.imshow("img", resized_gray)
        cv2.waitKey(0)

    maze = convert2list(resized_gray)
    print(maze)


    cv2.destroyAllWindows()

if __name__ == '__main__':
    img2binList(lenWidth=500.0, GRID_SIZE=20, verbose=1) # all unit is cm
                                                         # verbose to 1 for debugging
