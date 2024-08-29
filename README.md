# Navigation-Algorithms
Common navigation algorithms and useful tools that I've encountered or am interested in.

## Iterative Closest Point ##

### Description
For this project, I utilized a combination of Matlab and C++. The entire project is self-contained, utilizing a CMake file to compile. You'll need to have the [Eigen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page) to run it, so do keep that in mind.<br>
The Matlab file `generate_data.m` located in the /data/input_data folder pathway can be used to generate random data sets - one 'truth' and one 'measurement' data set. Upon running, this matlab file will automatically save off the entire Matlab workspace variable set as well as a .txt file of the two data sets (formatted appropriately for the C++ algorithm to pick up). From there, the built C++ project can be ran. Keep in mind that the .txt file's location needs to be updated within the `main.cpp` file.<br>
The final homogeneous transformation matrix produced via the Iterative Closest Point algorithm can be compared against a known truth version, which is stored off in the Matlab workspace data.
