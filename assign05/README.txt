
NAME: Bella Xia and Hanbei Zhou

Makefile shortcut: make / make cis
Excutable: cis

Library Imports:

To be able to run our program, we would need to include Eigen library in the include path. The version that we used
for our implementation is eigen-3.4.0

Instruction: 
To run the excutable on rebug or unknown data provided, type the following line in terminal:
    ./cis index
where index is A to H and K. When index is between A-F, the algorithm reads debug data; 
when index is between G, H ,and K, the algorithm reads unknown data.

    ./cis index char
where besides specifying index, we can also specify whether the linear search or the advanced (tree) search
would be used. Any character besides 'a' would indicate a linear search.

File description:

Helper functions / Main file

1. matrix.h, matrix.cpp:
With Eigen library (see the link in the end) needed, define the matrix object and implemente several calculations

2. rotation.h, rotation.cpp
Encapsulate the rotation information and error in matrix or angular/axis form.

3. position.h, position.cpp
Encapsulate the translation information and error in matrix form.

4. frame.h, frame.cpp
Encapsulate the frame transformation information. Define multiplication and inversion operation of frame.

5. registration.h, registration.cpp
Implemente the point cloud registration and pivot calibration operation.

6. interpolation.h, interpolation.cpp
Implemented the distortion correction function calculation and application.

7. io_read.h, io_read.cpp
Implemented methods for reading data from 8 types of file. Define f_data struct to store data of each frame. 

8. triangle_mesh.h, triangle_mesh.cpp
Implemented triangleMesh class to store a triangle. It calculates the closest point on
the triangle respect to a given point.

9. bounding_sphere.h, bounding_sphere.cpp
Implemented boundingSphere class to store the smallest sphere that contains a given triangle. 
It determines whether it is possible for the triangle inside to contain a closer point to a 
given matrix than a given bound.

10. bounding_box_tree_node.h, bounding_box_tree_node.cpp
Implemented boundingBoxTreeNode class to perform a more efficient bounding box search. 
It ignores all triangles that is impossible to contain a distance to a given matrix smaller than
the given bound.

11. mesh.h, mesh.cpp
Implemented the Mesh class that store all triangles and find the closest points among all the
triangles respect to a given point.

12. main.cpp
Produce runable file to calculate the EM distortion correction calculation and  and perform pivot calibration.

Unit Tests:

1. _matrix_test.cpp 
Test matrix-related operations such as assignments, plus / multiplication, transpose, and print

Makefile shortcut: make matrix
Excutable: matrix

2. _frame_test.cpp
Test rotation, translation and frame transformation related operations, including initiating
rotation / translation / frame, frame multiplication with matrices. 

Makefile shortcut: make frame
Executable: frame

3. _registration_test.cpp
Test the ability to initiate a point-cloud registration by inserting matrix lists a and b; test
functionalities point cloud registration and pivot calibration.

Makefile shortcut: make regis
Executable: regis

4. _io_test.cpp
Test reading into each type of input files (calbody, calreadings, empivot, optpivot)

Makefile shortcut: make io
Executable: io

5. _error_test.cpp
Test the inclusion of errors in both rotation and translation (not really used in programming assignment 1)
as well as their operations during frame multiplication

Makefile shortcut: make err
Executable: err

6. _calibration_test_1.cpp
Test a set case of pivot calibration.

Makefile shortcut: make calone
Executable: calone

7. _calibration_test_2.cpp
Test case with random rotational calibrations of a pivot with custom length and distance from reference frame.

Makefile shortcut: make caltwo
Executable: caltwo

8. _interpolation_test.cpp
Test interpolation implementation by comparing the expected and real value randomly selected from the given file.

Makefile shortcut: make interp
Executable: interp

9. _ct_frame_test.cpp
Test EM-to-CT frame transformation based on fixed values.
Makefile shortcut: make ct
Executable: ct

10. _ct_frame_test_2.cpp
Test randomized process of calibration; EM-to-CT frame registation based on calibrated result; transformation of new navigated EM into CT frame
Includes a random noise level of 0.001

Makefile shortcut: make ct2
Executable: ct2

11. _mesh_test.cpp

Create a randomized yz plane with points whose y and z coordinate would be in the triangle but would not be on the
same plane as triangle. So that if the point randomly generated is (.345, 0.567, 0.678), the system should output
(n, 0.567, 0.678) as the closest point, where n denotes the yz plane x = n which all the triangular meshes are in.

Makefile shortcut: make mesh
Executable: mesh

12. _mesh_test2.cpp

Create 3D ellipsoid surface, with the three radii
5.0, 10.0, and 15.0 respectively. We are able to feed a total of 5000 
vertices and triangular meshes into the Mesh class to create the 3D surface
feature. Then, we randomly generated 100 points on the surface and 
performed the search for optimum frame transformation. 
Mekefile shortcut: make mesh2
Executable: mesh2

13. _mesh_test3.cpp

same test as _mesh_test2 but with the inclusion of modes for each vertex

Mekefile shortcut: make mesh3
Executable: mesh3

Data Analysis

1. pa3_output_stats.cpp
Output statistics on the resulting output compard with expected output files

Makefile shortcut: make pa3-stats
Executable: pa3-stats

Use 5 as the second input in terminal indicates an overview over the
statistics of pa5
    ./pa3-stats 5


Reference
1. Eigen library:
https://eigen.tuxfamily.org/index.php?title=Main_Page
2. In registration.cpp (line 56), we referred to http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
to perform SVD.
3. In registration.cpp (line 71), we referred http://www.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche0062.html
to orthogonalize the result of SVD M to R.
