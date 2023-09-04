.EXE/Visual Studio:

- To run the compiled .EXE, you can run the CS550.exe shortcut. This is the latest build and will work the same way if you choose to build it in visual studio.
- I used Visual Studio 2022 to build the project, with the v143 version of tools. 
- In Visual Studio, please compile in Debug x64 for the best results.  There is a weird crash on startup in Release mode at the moment. It's related to one of the libraries I'm using. (Eigen linear algebra library).
- Other than that, the application is pretty stable. Haven't had any crashes related to my own code on this build (in Debug mode).


Application Instructions:

- Every Debug Drawing Option is enabled by default. Feel free to check/uncheck the boxes as needed.
- There's a Play/Pause button that controls the simulation. There's also a restart button that moves the objects back to their starting position. I'm clearing force accumulators so they shouldnt carry over any forces when this happens.
- You'll also find all the graphics options in the bottom if you feel like playing with that.

