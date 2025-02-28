CSCI 520, Assignment 1

SAI MANAS MUDA

================

For more details, Check the drive for clear documentation and demo videos: https://drive.google.com/drive/folders/1x-zE8BMaa_2CnSl5ZJqcxD3qIYxlE85L?usp=sharing


I was able to implement the jello cube simulation with mass spring system. Following are the important features we implemented:
-> Implemented spring force and damping force between all mass points with structural, shear and bend springs.
-> Implemented force field with trilinear interpolation
-> Implemented Bounding Box collision detection with normal and penetration depth calculation for penalty spring force.
    -> To detect collision I have checked if a mass point is outside all the planes. 
    -> Calculate plane equations for all normals  
-> Modified world structure to accommodate multiple objects in the world, so I have modified createWorld and readWorld functions too.
    -> World structure we can add multiple objects with different coefficients, positions and velocities.
    -> Implemented 'createObject' function in 'createWorld.cpp' which can be useful to createObject with various parameters I mentioned.
-> [*EXTRA*] Implemented OBB object collision detection between multiple objects. 
    -> To detect collision between objects, I check if mass point is between the object faces.
    -> I have calculated object's each face plane by calculating center using four corners and normal using face center and object center. Using the plane of all faces I calculated if mass point inside the object.
    -> To calculate normal and penetration length of collision I used OBB collision detection, by calculating a vector between two centers of objects and projecting on normal of face. If this projection length is greater than extend length of object in normal direction of face then this face is part of collisions. 
    -> We use the penetration depth and face normal after we decide face based on OOB Collision detection.
    -> For collision penalty spring I calculated coefficients by taking the average of the both objects.
-> [*EXTRA*] Implemented Raycast system on mouse left click.
    -> First I modified mouse input in 'input.cpp' such that I detect when mouse left click started and ended, so that I can raycast when mouse left click is started.
    -> For ray calculation based on mouse click (x,y) position on screen, first I retrieved camera projection matrix and modelView matrix using glut library, 
    -> Then I projected screen coordinates onto near plane and far plane of camera.
    -> I used nearPlane projection point as ray origin and difference between both points as ray direction.
-> [*EXTRA*] Implemented Mass Point selection using raycast and moving the mass point along normal to create deformations
    -> Using ray created using Mouse click, raycast is performed on all the objects.
    -> Ray is casted on all planes of the all objects and distance from ray origin is calculated for each ray casted on plane and then the least distance point is selected.
    -> Based on the point selected on face plane of object, I calculate nearest mass point and select that mass point for movement
    -> And when mouse is moved in Y-axis, the point is moved along normal of the plane.
-> [*EXTRA*] Implemented transparent red jello, which changes the light colors to red and and face material alpha to 0.5 using GLblend.

Keypoints on Implementation:
1) Structural Springs, Shear Springs, Bend Springs
2) Spring force and Damp Force on all springs
3) Force Field with trilinear interpolation
4) Bounding Box Collision Detection and penalty spring
5) Multiple objects simulation
6) Object(Jello) to Object(Jello) collision detection and penality spring
7) Raycast system to select mass point and move them manually
8) Transparent Red Jello


*** FEW THINGS TO KEEP IN MIND IN THIS PROJECT ***

-> I have modified world class to accommodate multiple objects in the scene, so the old testing world files will not work.
-> I have created new test case world files similar to old ones and also respective batch files to run those test cases in 'Executable folder'. Will be really useful to test the project.
-> Mouse Left click and hold on Cube to select a mass point, then move the mouse up and down to move the mass point. Release the left click to unselect mass point.
-> Click 'T' to change cube between transparent red jello and normal cube.
-> Animations folder has 3 types of animations stored, one with single cube, one with two cubes and one where I interact with jello manually by moving various mass points

