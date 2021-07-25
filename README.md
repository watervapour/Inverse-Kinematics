# Simulation on Inverse kinematic algorithms

This project aims to implement various methods of inverse kinematics. These will only be operating in 2D. This is written in C++, with SDL2 providing graphics and input support.

The first listed algorithms are dynamic, they only compare between each element, meaning they can run on long chains of segments. The kinematic solvers do not produce IK results, but are methods of calculating where the 'end effector' will be, based on the angles within the system.

Dynamic/runtime algorithms:
* [Coding Train](https://www.youtube.com/watch?v=RTc6i-7N3ms): An algorithm described in a youtube video from the coding train channel. This iterates from the tip to base, treating each segment as an individual piece. 
* [FABRIK](https://www.youtube.com/watch?v=UNoX65PRehA): Stands for "Forwards And Backwards Reaching Inverse Kinematics. This iterates from one end to the other, back and forth, smoothing out,similar to generating a bezier curve.

Kinematic solvers:
* [Homogeneous Transformation Matrix](https://www.youtube.com/watch?v=fXewWpehAWw&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&index=17): A special matrix is created that describes the armature system. 
* [Denavit Hartenberg](https://www.youtube.com/watch?v=4WRhVqQaZTE&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&index=18): A table of values is constructed.
