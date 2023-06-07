# Robotic-Manipulator-for-Chocolate-Boxes-Packing
![de](https://github.com/Picardo31/Robotic-Manipulator-for-Chocolate-Boxes-Packing/assets/70179309/db96099e-589d-42fd-9248-1c02a04d76de)
The idea of this project is to create just one robotic arm that can do the process of storing and packing the product, instead of having a line of robots with different tasks. This means that the user can leave an empty box and indications of storage to the robot. In this way the robot arm will start to grab each chocolate one by one and place them in an empty position inside the packing box. In this way, it is proposed to carry out a movement algorithm capable of establishing point coordinates of positions considered empty, where the robot arm will place the product, in order to determine occupied positions, and later discard them and place the new products in unoccupied positions.
Geometric Analisys:
![Diagrama](https://github.com/Picardo31/Robotic-Manipulator-for-Chocolate-Boxes-Packing/assets/70179309/432c7d8f-651f-4126-8552-b5157a7eb816)

Features:
* Forward Kinematics
* Inverse Kinematics implemented with geometric analisys and genetic algorithms ± 2 mm
* Gripper with adaptable elastic band for uneven surfaces
* Automatic and Manual mode
* Control of position, velocity and acceleration
* Controlled by KY-022 infrared sensor and LCD display
* 3:1 gear system for each actuator
* Built with arduino Mega, Shield, Nema 17 and drivers DRV8825
