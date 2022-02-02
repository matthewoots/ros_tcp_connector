# Trajectory Visualization
sudo apt-get install -y ros-melodic-hector-trajectory-server

# Ceres
cd 
mkdir ceres_solver
cd ceres_solver
wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j3
make install
