This is an IMU-Preintegration library which can be used for VIO pipeline. It has been tested on EUROC dataset provided under data folder

Related Publications:
Forster C, Carlone L, Dellaert F, et al. On-Manifold Preintegration for Real-Time Visual--Inertial Odometry. IEEE Transactions on Robotics, 2017, 33(1): 1-21. PDF.

To run this library:

git clone https://github.com/ujasmandavia/IMU_Preintegration.git \
cd IMU_Preintegration \
mkdir build \
cd build \
cmake .. \
make -j4

Don't forget to change the path to your dataset

This dataset is available online too \
https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

