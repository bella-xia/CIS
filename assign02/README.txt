
NAME: Bella Xia and Hanbei Zhou

Makefile shortcut: make / make cis
Excutable: cis

Library Imports:

To be able to run our program, we would need to include Eigen library in the include path. The version that we used
for our implementation is eigen-3.4.0

Instruction: 
To run the excutable on rebug or unknown data provided, type the following line in terminal:
    ./cis index
where index is a to k. When index is between a-g, the algorithm reads debug data; 
when index is between h-k, the algorithm reads unknown data.

To run the excutable on other source files, type the following line in terminal:
    ./cis other
the program will then prompt you to enter the source file names.

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

7. io_read.h, io_read.cpps
Implemente methods for reading data from 4 types of file. Define f_data struct to store data of each frame. 

8. main.cpp
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

Data Analysis

1. output_comp.cpp
Output statistics on the resulting output compard with expected output files

Makefile shortcut: make comp
Executable: comp

Using custom file and its corresponding output file on C expected:
    ./comp <expected output filename> <student outpu filename>

Using the current a-g debug output files on C expected:
    ./comp C

Using the current a-g debug output files on pivot calibration:
    ./comp

Due to the need for repeated trials to be able to obtain a range of possible values for pivot calibration based on
which frame is chosen to be used to determine the frame axis (G_0), we are unable to provide a quick access to
stastics over pivot based on the expected and actual output files alone.



Reference
1. Eigen library:
https://eigen.tuxfamily.org/index.php?title=Main_Page
2. In registration.cpp (line 56), we referred to http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
to perform SVD.
3. In registration.cpp (line 71), we referred http://www.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche0062.html
to orthogonalize the result of SVD M to R.
