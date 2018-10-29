# EECS149 project - Detective Kobi

### Team Members: Elvis Chau, Pairode Jaroensri, Joni Leung, Titlis Wan

### Project Goal

This project will help Kobuki locate and find a potentially moving ultrasonic beacon, while detecting and avoiding obstacles in a distance. 

### Project Approach

This project will be separated into two parts. Helping Kobuki locate the beacon using ultrasonic detectors, and implementing a sensor to map the surrounding and helps Kobuki to avoid bumping into any obstacles while it is locating the beacon. The mapper might be used to implement more functionalities as we come up with one. To locate the beacon, we will use two ultrasonic detectors mounted on the opposite sides of the Kobuki, which will function similar to human ears. We can tell which direction the beacon is by calculating the phase shifts of the ultrasonic between the two sensors. As for the mapper, we will use a time-of-flight sensor mounted on a servo. The sensor will be continuously scanning the room at different angles in the similar manner as RADAR.
