# Path-Planning-for-point-robot-using-A-star-search-algorithm

## Project Description

A Star search algorithm was predominantly used as a path planner for autonomous robots a while ago. It has some advantageous characteristics like finding the path with least cost. In this project we will be using A star algorithm to plan the path for a mobile robot in an environment with static variables.

## Objective

Implement A* Algorithm to find a path between start and end point on a given map for a mobile robot. We need to make sure that no state is repeated so as to avoid infinite looping.

* **[A Star](https://brilliant.org/wiki/a-star-search/)** : A* is an informed search algorithm, or a best-first search, meaning that it is formulated in terms of weighted graphs starting from a specific starting node/state, it aims to find a path to the given goal node having the smallest cost (least distance travelled, shortest time, etc.).
* **Action/Move Set** : The Action/Move Set that the algorithm uses has 5 actions each having a Euclidean distance threshold of 0.5 unit and a theta threshold of 30 degrees for each action. Using these actions, the algorithm generates new states at every state and checks whether each new state is the required `goal state`.
* **Distance Metric** : We are using the Euclidean Distance as the Metric which is the absolute distance between two points. It's being used with a threshold of 0.5 unit. Euclidean distance is used as a threshold to constraint the maximum extent of actions.
* **Unique State Checker** : We implement a set data structure to store all existing states. We will write an algorithm such that every unique state that is being entered into the set will be compared to all other existing states to avoid repitition.

## Contents

<pre>
    ├── Outputs  
    │   └── images
    ├── LICENSE
    ├── README
    └── A-star_path_planner.py
</pre>

## Instructions

### Flowchart of the algorithm

<p align="center">
<img width="50%" alt="TB3" src="https://user-images.githubusercontent.com/106445479/215384762-67b34d31-5ae9-40c7-8104-3e14a8ae34f3.png">
</p>

### Steps to run my implementation

1. Clone the repository
```
git clone https://github.com/bharadwaj-chukkala/Path-Planning-for-a-point-robot-using-A-star-Search-Algorithm.git
```
2. Install Python 3.9 and the libraries mentinoned below prior to running the code
3. Go to the root directory from your IDE.
4. Open the clonned project file in any IDE. (I used PyCharm)
5. UnComment the line used for Visualization i.e. '309', which says- "plt.pause(0.00...1)"
6. Run the Program
7. In the Console, the program asks for:
   * The Obstacle Clearance and Robot Radius
   * Robot Step Size and
   * The x and y coordinates of Start and Goal Node.
8. Enter as prompted. example below
   1. Clearance: 5
   2. Robot Radius: 10
   3. Step size: 1
   4. Start: 50, 50
   5. Initial Theta: 30
   6. Goal: 150, 150
   7. Final Theta: 30.
9. The Output Plot with planned Path should be Visible.

### Dependencies

* NumPy
* argparse
* Matplotlib
* math
* heapq
* time

## Results

#### Understanding the Output Plots

- The Robot movable space is shown in White Color
- The Pixels in Blue are the Obstacles.
- The Pixels in Cyan is the Clearance Space.
- The explored path is marked by Green color.
- The Planned Path is shown by Red Dotted Lines.

<p align="center">
<img width="40%" alt="Step size 1" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-point-robot-using-A-star-Search-Algorithm/blob/main/Outputs/images/step_size%201.png"> <br>
<b>Step size 1</b>
</p>

<p align="center">
<img width="40%" alt="Step size 2" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-point-robot-using-A-star-Search-Algorithm/blob/main/Outputs/images/step_size%202.png"> <br>
<b>Step size 2</b>
</p>

<p align="center">
<img width="40%" alt="Step size 3" src="https://github.com/bharadwaj-chukkala/Path-Planning-for-a-point-robot-using-A-star-Search-Algorithm/blob/main/Outputs/images/step_size%203.png"> <br>
<b>Step size 3</b>
</p>

### [Implementation Video Drive Link](https://drive.google.com/drive/folders/1o5HvvbLppkN7Pmss0JQCHE8xHmnWHCw_?usp=sharing)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author Contact

**Bharadwaj Chukkala** <br>
UID: 118341705 <br>
Bharadwaj Chukkala is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include Machine Learning, Perception and Path Planning for Autonomous Robots. <br>
[![Contact](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](bchukkal@umd.edu)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/bharadwaj-chukkala/)
[![GitHub](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/bharadwaj-chukkala)
