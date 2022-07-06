In ~/.bashrc:

`export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so:/usr/lib/x86_64-linux-gnu/libGL.so`

Also, ensure the NVIDIA drivers are set to default, not Nouveau

These changes avoid the common `GLEW initialization error` and `Segmentation Fault`