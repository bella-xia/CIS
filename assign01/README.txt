
NAME: Bella Xia and Hanbei Zhou

Excutable: cis
Instruction: 
To run the excutable on rebug or unknown data provided, type the following line in terminal:
    ./cis index
where index is a to k. When index is between a-g, the algorithm reads debug data; 
when index is between h-k, the algorithm reads unknown data.

To run the excutable on other source files, type the following line in terminal:
    ./cis other
the program will then prompt you to enter the source file names.

File description:

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

6. io_read.h, io_read.cpps
Implemente methods for reading data from 4 types of file. Define f_data struct to store data of each frame. 

7. main.cpp
Produce runable file to calculate the C_i and perform pivot calibration.



Reference
1. Eigen library:
https://eigen.tuxfamily.org/index.php?title=Main_Page
2. In registration.cpp (line 56), we referred to http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
to perform SVD.
3. In registration.cpp (line 71), we referred http://www.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche0062.html
to orthogonalize the result of SVD M to R.
