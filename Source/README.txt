/* Start Header -----------------------------------------------------------------
Copyright (C) 2019 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.
File Name: README.txt
Purpose: Describes the project
Project: brady.m_CS350_4
Author: Brady Menendez, brady.m, 180000316
Creation date: 4/25/19
End Header --------------------------------------------------------------------*/

a. How to use parts of your user interface that is NOT specified in the assignment description.
GJK COLLISION DETECTION section:
- Use the DRAW ITERATIVE STEPS checkbox to toggle the drawing of the iterative steps of the gjk algorithm (the refinement of the simplex).
It's on by default.
- Press SPACE to launch a sphere in the direction you're looking at. Once it collides with any of the 6 objects in the scene , if the draw checkbox is ticked, it'll start drawing the first 
simplex, to move to the next one, just keep pressing SPACE. If the checkbox is not ticked, the sphere will just dissappear once it collides.

NOTE: the application starts unlit, so clicking on any of the light scenarios under the LIGHT SCENARIO SELECT section will turn on all lights to see 
the objects.

b. Any assumption that you make on how to use the application that, if violated, might cause	
the	application	to	fail.
Build Solution in x86 mode, I used 32 bit libraries. Also the solution probably needs to be retargeted.

c. Which part of the assignment has been completed?	
All of it.

d. Which  part  of the assignment has NOT been completed (not	 done,	 not	 working,	 etc)	 and	
explanation	on	why	those	parts	are	not	completed?
None, to my knowledge.

e. Where the relevant source codes (both C++  and  shaders)  for  the assignment are located.	
Specify	the file path (folder name), file name,	function name (or line number)
The shaders are under Shaders/
Rest of the source files can be found in the root folder:
- main.cpp (Lines: 716, 866-963, Functions: StepCollision, SpawnSphere, UpdateSphere, DrawSimplex)
- BoundingUtils.cpp/.h (Lines: 1137-1448, Functions: DetectCollision_BroadPhase, DetectCollision_MidPhase, 
DetectCollision_GJK, NearestSimplex and a bunch of helper functions);

f. Which machine (or lab) in DigiPen that you test your application on
DIT2641US

g. The  number of hours you spent on the assignment, when you started working on it and
when you completed it.
12 hours

h. Any	other	useful	information	pertaining	to	the	application	
None really, everything should be pretty straight forward.