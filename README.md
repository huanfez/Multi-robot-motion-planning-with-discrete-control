# Multi-robot-motion-planning-with-discrete-control
Symbolic motion planning for multi-robot collision avoidance problem. The scenarios are 3 agents move in 5 five different environments. The collision was solved by decomposing concatenated inter collision situations and predifining all the possible atomic inter collision behaviors.

Setup:
You need to setup the environment variables in your computer for NuSMV
enter into the system and user environment variables interface, add the NuSMV include and bin

user Variables --> new 
variable: NUSMV_LIBRARY_PATH
value : "Your NuSMV directory"\NuSMV-2.5.4-x86_64-w64-mingw32\share\nusmv

system Variables -->path -->edit
add new by
"Your NuSMV directory"\NuSMV-2.5.4-x86_64-w64-mingw32\bin

Note: After this step, the NuSMV should not be moved or deleted
