Submitters: Guy Madmon, Ido Shimshi 

Essential Game Features:

 - Snake with maximum 16 links (can change it, its a constant in BasicScene.h)
 - There are three cameras: First person, third person and static camera
 - Menu - At the start of each game, at the end of each game, and a responsive one mid game
 - Skinning - There is a skinning feature to the snake using Dual Quaternion method.
 - Collision Detection - The snake can collide with points in 3d, with swords, and with it self (the last two cause the loss of the game)
 - Scoring mechanism - There is a scoring mechanism for each level. It is descibed within the game at the start of each level.
 - Basic Locomotion - The snake moves in space and its' links follow it

Extra Features:
 - Texture - The snake has snake skin texture, also its tongue, the eggs and the sword.
 - Sound - Sound clips are being played on different occasions, like getting point, getting hit by sword etc..
 - Interactive User Interface - Showing the score mid game, showing the time remaining / time passed (according to the current level)
 - Prevent self collision - If the snake hits it self you lose the game
 - In the third level, the eggs are slowly fading out (using shader)
 - You can move in two different directions simultaneously by pressing two buttons at a time (e.g press W and A)

Levels:
 - Level 1:
    You must get all 12 eggs in time (120 seconds). The snake grows each egg you eat.
    You start with 4 links.
    You have special ability: You can move faster by pressing 'e' for a cetain amount of time
 - Level 2:
    You must get all 12 eggs. The snake grows each egg you eat.
    You must also dodge the moving swords.
    You start with 4 links.
    You have special ability: You can move faster by pressing 'E' for a cetain amount of time
    You have another special ability: You can make yourself invisible by pressing 'Q' for a cetain amount of time making you involnerable to the sword.
- Level 3:
    You must get all 12 eggs. The snake grows each egg you eat.
    The eggs vanish after 20 seconds (also slowly disappearing through shader).
    The score you get is relative to the amount of time took you to reach the egg.
    You start with 4 links.
    You have special ability: You can move faster by pressing 'E' for a cetain amount of time

Difficulties:
 - We had a real difficulty at doing the skinning, like realizing the vertex coordinate system is according to the mesh and not the model, positioning the
   joints etc..
 - Adding a library for the sound feature was hard
 - The basic locomotion of the snake - making the joints follow one another was hard, working with quaternions and so on..