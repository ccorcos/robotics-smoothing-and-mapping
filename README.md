Smoothing and Mapping
---

This script uses pylab to simulate robots with sensors moving in an
environment simultaneously localizing and mapping. 

You can define your own motion models and sensor models, and record
the robot(s) moving though a simulated 2D environment. 

Currently, the code is a mess because I had a project deadline. Refer
to "NOTES" for more information.

I have written my own nonlinear graph optimizer in python for this 
project. However, rather than use colamd and QR, I have just 
implemented pinv which is inefficent and does not leverage the 
sparsity of the problem.

Room for improvement:

*   create other sensor models (GPS)
*   create other motion models (gantry)
*   helper function for creating robots
*   generalize for multiple robots (merge graph)
*   colamd and QR for graph optimization
*   implement graph optimization in the robot class (as opposed to the simulator)