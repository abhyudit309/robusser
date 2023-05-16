# Robusser: Dish Washing Robot

<p align="center">
<img width="600" alt="Screen Shot 2023-05-09 at 11 59 48 PM" src="https://github.com/dihim/robusser/assets/57520931/62565511-ca41-4214-b60f-fcd865f6f3d4">
</p>

## Project Overview 
**Robot Task:** Dish Washing
 - Given a table containing dishes in a room, the robot will:
 - Navigate the room
 - Pick up the used dishes
 - Put them in the sink
 - Pick them up from the rinsed dishes mat
 - Then, put them in the dishwasher
 - Return to charging state

**Robot Design:**
 - Single moving base with one 7-DoF arm attached for picking up arbitrarily positioned objects
Arms positioned such that they can reach both the table top and dishwasher down below
 - Two cameras to aid in haptic control
   - Robot’s point of view 
   - Depth of robot gripper

## Installation Instructions
Simply clone this repository in the directory corresponding to the cs225a repository.
```
cd cs225a
git clone https://github.com/dihim/robusser.git
cd robusser
```
