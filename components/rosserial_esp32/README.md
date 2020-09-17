# rosserial_esp32

ESP-IDF component to connect to ROS server.

## Auto generation of ros_lib directory with C++ header files.
 
Content of `ros_lib` directory is generating from ROS messages files manually. 

Run script:

```
bin/generate_ros_lib.sh
```

to run Docker container with ROS and generate 
C++ header file base on ROS messages files from packages installed to 
ROS.