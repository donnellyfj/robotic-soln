# Robotic_HW
## To Run
### Install ROS2
Install instructions (Ubuntu): [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2 tutorials: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

### Run the Solution (Linux)
1. Clone this repository
2. Source your environment and build the project (instructions on how to do this can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html))
3. Run ```python3 sensor.py``` to start the sensor simulator servers. You may need to change the port values if the defaults are in use. Make sure to make the same port changes in service_member_function.py
4. In a new terminal, run ```ros2 run py_srvcli service``` to start the service node
5. In a new terminal, run ```ros2 run py_pubsub listener``` to start the subscriber node
6. In a new terminal, run ```ros2 run py_srvcli client``` to start the client node
7. You should now be able to see the client make calls to the service and publish the results to a topic, which will be reflected in the subscriber terminal
8. CTRL + C each of the terminals to stop execution
