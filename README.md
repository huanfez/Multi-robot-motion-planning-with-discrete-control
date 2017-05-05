# Multi-robot-motion-planning-with-discrete-control
Symbolic motion planning for multi-robot collision avoidance problem
1)You need to setup the environment variables in your computer for NuSMV
enter into the system and user environment variables interface, add the NuSMV include and bin

user Variables --> new 
variable: NUSMV_LIBRARY_PATH
value : "Your NuSMV directory"\NuSMV-2.5.4-x86_64-w64-mingw32\share\nusmv

system Variables -->path -->edit
add new by
"Your NuSMV directory"\NuSMV-2.5.4-x86_64-w64-mingw32\bin

Note: After this step, the NuSMV should not be moved or deleted

2) Modify the line 111 in the main file "ExAbsCollabTest_RT_v5.m" to be
filePath = 'Your NuSMV directory\LiveLock\SymCodes';
