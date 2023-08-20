# CS225A Final Project: Robusser - Dish Washing Robot

See [`CS225A Final Project Report - Robusser.pdf`](https://github.com/abhyudit309/robusser/blob/main/CS225A%20Final%20Project%20Report%20-%20Robusser.pdf) and [`Final Presentation.pptx`](https://github.com/abhyudit309/robusser/blob/main/Final%20Presentation.pptx) for a more detailed description of the project implementation.

## Project Overview 

* **Robot Task**: Dish Washing
* **Motivation**: Assistive Care
* Given a table containing dishes in a room, the robot will:
  - Navigate the room
  - Pick up the used dishes and cups
  - Rinse them in the sink
  - Put them in the dishwasher
  - Return to the charging state
* **Robot Design**:
  - Single **moving base** with **one 7-DoF arm** attached for picking up arbitrarily positioned objects
  - Arm positioned such that it can reach both the table top and dishwasher down below

### Project Video

https://github.com/abhyudit309/robusser/assets/71165429/324d3cc2-cbdc-431e-ad6c-f4513215b94c

## Installation Instructions
Simply clone this repository in the directory corresponding to the cs225a repository:

```bash
cd cs225a
git clone https://github.com/abhyudit309/robusser.git
cd robusser
```
