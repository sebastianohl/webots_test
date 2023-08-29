# Grid-based Height Fusion

This projects provides you the same setup as in the previous challenge. However, this time you will implement a height grid fusion. The goal is to measure the height above ground of the boxes based on a downwards looking LIDAR sensor. The result is stored in a earth fixed grid data structure.

## Task
* Decide which programming language you want to use and change the `fusion.package` in `uav_launch/launch/robot_launch.py`
* Read through the code in `uav_fusion|uav_fusion_cpp` and try to understand its purpose
* This is an earth fixed grid. Therefore you need to move it in `move` function
* Implement a fusion algorithm in `updateCell`. Select an algorithm that sounds reasonable to you for a height value.
* Implement the preprocessing of the incoming sensor data in `onLidar`. The data comes in sensor coordinates. You want to convert it to earth fixed world coordinates. Therefore, it should not move with your UAV and also need to be compensated for rotation around any axis. The Z-value should have the value above ground (e.g. if the box is 1m in height, Z=1.0). The 0.0 values does not make sense and need to be replaced.

## Hints
* Do one step after the other!
* Compile and run your code often. So you only need to debug small parts.
* Wait for complete startup. 
* Sometimes the first startup is not working and `launch.sh` is terminating. Try again
* Sometimes the first startup does not have 3D acceleration. Try again

## Upload
* Zip your complete project on the console by running `zip -r upload-myname.zip .`
* Do a screen recording of your 3D view with your favorite screen recorder
* Upload all to StudIP
