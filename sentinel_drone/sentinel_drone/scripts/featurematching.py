import cv2 
import matplotlib.pyplot as plt
from osgeo import gdal
#matplotlib inline
import subprocess


# read images
img1 = cv2.imread('aerial.png')  
img2 = cv2.imread('task2d.tif') 

img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)


# ORB Detector
orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# img1kp=cv2.drawKeypoints(img1,kp1,color=(0,255,0),flags=0)
# img2kp=cv2.drawKeypoints(img2,kp2,color=(0,255,0),flags=0)
# cv2.imwrite('m_img1.jpg',img1kp)
# cv2.imwrite('m_img2.jpg',img2kp)
# Brute Force Matching
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)
matches = sorted(matches, key = lambda x:x.distance)

#img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[50:], img2, flags=2)
#plt.imshow(img3),plt.show()

kp1 = list(kp1)
kp2 = list(kp2)
# print(kp1[1],kp2[1])
# print(len(kp1))
# print(len(kp2))
# print(type(matches))

# Initialize lists
list_kp1 = []  #contains all thepixel corrdinated of matched featured(keypoints)
list_kp2 = []  #contains all the pixel corrdinated of matched featured(keypoints)

# For each match...
for mat in matches:

    # Get the matching keypoints for each of the images
    #queryIdx and trainIdx tell you which ORB features match between the first and second image
    #queryIdx - The index or row of the kp1 interest point matrix that matches
    #trainIdx - The index or row of the kp2 interest point matrix that matches
    img1_idx = mat.queryIdx
    img2_idx = mat.trainIdx

    # x - columns
    # y - rows
    # Get the coordinates
    (x1, y1) = kp1[img1_idx].pt
    (x2, y2) = kp2[img2_idx].pt
    #list_kp1 will contain the spatial coordinates of a feature point that matched with the corresponding position in list_kp2

    # Append to each list
    list_kp1.append((x1, y1))
    list_kp2.append((x2, y2))

#print(list_kp1[1])
#print(list_kp2[1])
# plt.imshow(img3),plt.show()
# plt.imshow(img1),plt.show()
#plt.imshow(img2),plt.show()

# Open tif file
ds = gdal.Open('task2d.tif')
# GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
xoff, a, b, yoff, d, e = ds.GetGeoTransform()
print(xoff, a, b, yoff, d, e )
  
def pixel2coord(x, y):
    """Returns global coordinates from pixel x, y coords"""
    xp = a * x + b * y + xoff
    yp = d * x + e * y + yoff
    return(xp, yp)
  
# get columns and rows of your image from gdalinfo
# rows = 4001
# colms = 3994
# long_lat = []
# for row in  range(0,rows):
#         for col in  range(0,colms): 
#             #print (pixel2coord(col,row)) #longitude , #latitude
#             a,b = pixel2coord(col,row)
#             long_lat.append((a,b))
# print(len(long_lat))

#print(list_kp2)
#print(len(list_kp2))
#print(len(matches))
kp_lat_long = []   #will contain all the longitude adn latitude of matched feature points


for i in range(0,len(list_kp2)):
    kplong,kplat = pixel2coord(list_kp2[i][0],list_kp2[i][1])
    kp_lat_long.append((kplong,kplat))

print(len(kp_lat_long))
cmd_list = ["gdal_translate","-gcp","list_kp1[1][0]","list_kp1[1][1] ","kp_lat_long[1][0]" ,"kp_lat_long[1][1]","-gcp","list_kp1[2][0]","list_kp1[2][1] ","kp_lat_long[2][0]" ,"kp_lat_long[2][1]","-gcp","list_kp1[3][0]","list_kp1[3][1] ","kp_lat_long[3][0]" ,"kp_lat_long[3][1]","-gcp","list_kp1[4][0]","list_kp1[4][1]","kp_lat_long[4][0]" ,"kp_lat_long[4][1]","-gcp","list_kp1[5][0]","list_kp1[5][1] ","kp_lat_long[5][0]" ,"kp_lat_long[5][1]","-gcp","list_kp1[6][0]","list_kp1[6][1] ","kp_lat_long[6][0]" ,"kp_lat_long[6][1]","-gcp","list_kp1[7][0]","list_kp1[7][1] ","kp_lat_long[7][0]" ,"kp_lat_long[7][1]","-gcp","list_kp1[8][0]","list_kp1[8][1] ","kp_lat_long[8][0]" ,"kp_lat_long[8][1]","-gcp","list_kp1[9][0]","list_kp1[9][1] ","kp_lat_long[9][0]" ,"kp_lat_long[9][1]","-gcp","list_kp1[10][0]","list_kp1[10][1] ","kp_lat_long[10][0]" ,"kp_lat_long[10][1]","-gcp","list_kp1[11][0]","list_kp1[11][1] ","kp_lat_long[11][0]" ,"kp_lat_long[11][1]","-gcp","list_kp1[12][0]","list_kp1[12][1] ","kp_lat_long[12][0]" ,"kp_lat_long[12][1]","-of","GTiff", "aerial.png", "map-with-gcps.tif"]
subprocess.run(cmd_list)

# cmd_list2 = ["gdalwarp","-r", "bilinear", "-s_srs","EPSG:4326", "-t_srs", "EPSG:4326", "-overwrite", "map-with-gcps.tif","map-with-gcps-change-reference.tif"]
# subprocess.run(cmd_list2)

# Open tif file
ds1 = gdal.Open('map-with-gcps.tif')
# GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
xoff1, a1, b1, yoff1, d1, e1 = ds1.GetGeoTransform()

def pixel2coord1(x1, y1):
    """Returns global coordinates from pixel x, y coords"""
    xp1 = a1 * x1 + b1 * y1 + xoff1
    yp1 = d1 * x1 + e1 * y1 + yoff1
    return(xp1, yp1)

pixel2coord1()








