ANN/DMM Quadruped Tests:
------------------------

This experiment evolves ANN/DMM controllers for a quadruped robotic platform, or ANN only controllers by running the file located in ./ann\_only.  Experiments can be run from the launch.sh file with the call of the form:

./launch\_runs <start_run_num> <end_run_num> &

I recommend using a screen environment if deploying remotely so that you can view the status of runs when logging back in to your system.  The code is threaded and will use up to n-2 cores on the system with n being the number of cores available.  

### Installing ODE:
Prior to running the code, you must first build ODE and install the Python bindings.  Instructions for doing so can be found at: http://ode-wiki.org/wiki/index.php?title=Manual:\_Install\_and\_Use

The original experiments were run with --enable\_double\_precision and --disable\_asserts

### MultiNEAT:
The MultiNEAT package will also need to be built.  A few modifications have been added by myself to allow for tracking the ANN component and attaching it to a DMM controller.  This modified code can be found and built in the /Controllers/MultiNEAT folder of this software.
