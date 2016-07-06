# Memo for localization development

## Aruco

[OpenCV Documentation on Aruco](http://docs.opencv.org/master/d4/d17/namespacecv_1_1aruco.html#gsc.tab=0)

This may not be a good solution because, 
 - Identifying different markers is not necessary.
 - Because of the structure of the marker, it has to be big enough to be detected.
 - The computation is quite heavy for RaspberryPi Zero.
 - It relies on edges, so it is sensitive to motion blur.