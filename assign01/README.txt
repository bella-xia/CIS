TODO:
1. citation
2. change the absolute file pathway to relative

Excutable: cis
Instruction: 
To run the rebug or unknown data provided, type:
    ./cis index
where index is a to k. When index is between a-g, the algorithm reads debug data; 
when index is between h-k, the algorithm reads unknown data.

File description:

1. matrix.h, matrix.cpp:
With Eigen library #citation needed, define the matrix object and implemente several calculations

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

