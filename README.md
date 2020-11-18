# Linkwiz Test

The task is to identify the radii, centers and normals of the two holes. We are given a txt file containing an unorganised pointcloud.

1. We proceed first by converting the txt file to a PCD format for easier usage by the PCL Library.
2. We then shift the entire point cloud to the origin, by subtracting the centroid coordinates. This is done because concaveHull needed in the next step performed very poorly, without shift to origin. We experiment with rotation too, but find it to be an overkill.
3. We then use the concaveHull library to find the concaveHull representing the boundary of the figure. We had to experiment with the parameters to get the fit.
4. The concaveHull is then used to find the circles - we use CIRCLE_3D for getting the first large circle. After that, it was cumbersome to get the second circle directly, because of its small size and the distance from the edges (the parameter tuning was taking more time), so we fitted a 2D circle on the entire large rectangle, and removed most part of it. Once that was done, we easily fitted a CIRCLE_3D for the smaller circle too.

To run code:
~~~
mkdir build
cd build
cmake ..
make
./convertFile //for converting the file
./shiftCentreToCentroid //for shifting the centroid
./findConcaveHull //for finding the hull
./findCircles //for finding the circles

~~~

The transformation matrix was: (the centroid is the last column multiplied by -1)
~~~
       1        0        0 -63.1533
       0        1        0  113.397
       0        0        1  28.1117
       0        0        0        1
~~~

The coordinates of the circle after shift of the origin (finding values), and then adding the centroid are (all values in cm):
~~~
--------------- INFORMATION -------------
CIRCLE 1
w.r.t x_centroid: 4.83283	 w.r.t x_origin: 67.9861
w.r.t y_centroid: -16.1884	 w.r.t y_origin: -129.585
w.r.t z_centroid: -6.70003	 w.r.t z_origin: -34.8117
Radius (cm): 8.84055
Normal_x: 0.121545
Normal_y: -0.348384
Normal_z: 0.929438
------------------------------------
CIRCLE 2
w.r.t x_centroid: 3.14926	 w.r.t x_origin: 66.3026
w.r.t y_centroid: 14.7879	 w.r.t y_origin: -98.6091
w.r.t z_centroid: 5.1311	 w.r.t z_origin: -22.9806
Radius (cm): 2.60824
Normal_x: -0.121545
Normal_y: 0.348384
Normal_z: -0.929438
~~~