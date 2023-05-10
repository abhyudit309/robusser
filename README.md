# Robusser: Dish Washing Robot

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
