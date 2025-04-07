import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

maze_length = 410
maze_width = 470
brick_length = 36
brick_width = 11

img = cv.imread(r'C:\Users\riann\Downloads\maze4.jpg')   # you can read in images with opencv
gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
resized_image = cv.resize(gray, (maze_width, maze_length)) 
img = resized_image

blur= cv.GaussianBlur(gray,(7,7),0)
edged = cv.Canny(image= img, threshold1=45, threshold2=200, apertureSize=3)

# Pixelation
height, width = img.shape[:2] # Get input size
w, h = (47, 41) # Desired "pixelated" size
temp = cv.resize(edged, (w, h), interpolation=cv.INTER_AREA) # Resize input to "pixelated" size
output = cv.resize(temp, (width, height), interpolation=cv.INTER_AREA) # Initialize output image
ret, thresh = cv.threshold(output,0,255,cv.THRESH_OTSU)

contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
print("Number of Contours found = " + str(len(contours))) 
cv.drawContours(img, contours, -1, (0, 255, 0), thickness=cv.FILLED) 


mask = np.ones(img.shape[:2], dtype="uint8") * 255
cv.drawContours(mask, contours, -1, 0, -1) # Draw the contours on the mask

# Pixelation
height, width = mask.shape[:2] # Get input size
w, h = (20, 15) # Desired "pixelated" size
temp2 = cv.resize(mask, (w, h), interpolation=cv.INTER_AREA) # Resize input to "pixelated" size
output2 = cv.resize(temp2, (width, height), interpolation=cv.INTER_AREA) # Initialize output image
ret, thresh2 = cv.threshold(output2,0,255,cv.THRESH_OTSU)

plt.imshow(thresh2)
plt.show()

thresh3 = np.asmatrix(thresh2)
print(thresh3)

wow = []
counter = 0

# rows = len(thresh3)
# col = len(thresh3[0])
row = np.shape(thresh3)[0]
col = np.shape(thresh3)[1]
print(row)
print(col)

# for i in range(row):
#     for j in range(col):
#         if thresh3[i][j] == 255:
#             thresh3[i][j] = 1
        #     if counter == 3:
        #         wow.append(0)
        #         counter = 0
        # if j == 255:
        #     counter = counter+1
        #     if counter == 3:
        #         wow.append(1)
        #         counter = 0

print(thresh3)
