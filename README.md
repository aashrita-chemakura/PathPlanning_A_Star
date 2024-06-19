# Path Planning with A-star Algorithm

This repository hosts the implementation of the A-star algorithm tailored for a mobile robot navigation system, developed as part of the ENPM661 course project. The algorithm efficiently finds the shortest path from a start to a goal position considering the robot's orientation and physical constraints.

## Authors

- [Suriya Suresh](https://www.github.com/theunknowninfinite)
- [Aashrita Chemakura](https://github.com/aashrita-chemakura)

## Project Goals

The primary objective of this project is:
- To implement the A-star Algorithm for autonomous path planning of a mobile robot.

## Python Packages Used

The project utilizes the following Python libraries:

- numpy
- opencv (cv2)
- time
- heapq
- matplotlib (matplotlib.pyplot)

## Setting Up the Script

To get the script running on your machine, follow these steps:

1. Clone the GitHub repository:
    ```bash
    $ git clone https://github.com/aashrita-chemakura/PathPlanning_A_Star.git
    ```

2. Navigate to the repository directory and run the script:
    ```bash
    $ python a_star_jayasuriya_Aashrita.py
    ```

## Providing Inputs

Upon execution, the script will prompt you to enter:
````
1. Initial coordinates of the robot (format: x, y)
2. Initial orientation of the robot (in degrees)
3. Goal coordinates of the robot (format: x, y)
4. Orientation of the robot at the goal point (in degrees)
5. Step Size (integer)
6. Radius of the robot (integer)
7. Clearance (integer)
````
**Note :** Ensure the total of clearance and radius does not exceed 10 to avoid sizing issues with the robot's model.

## Outputs

The script will display:
- A plot representing the path taken by the robot from the start to the goal point. This visualization helps in understanding the path planning process and the effectiveness of the A-star algorithm.

## Notes

Ensure all dependencies are installed before running the script to avoid runtime errors. You may install the required libraries using pip:
```bash
pip install numpy opencv-python matplotlib
````
