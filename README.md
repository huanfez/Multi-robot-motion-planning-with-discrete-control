# Multi-agent-motion-planning-with-discrete-control
Symbolic motion planning for multi-agent collision avoidance problem. 
The scenarios are 3 agents move in 5 five different environments. The collision was solved by predifining all the possible atomic inter collision behaviors and decomposing concatenated inter collision situations.

Setup: Add the environment variables for NuSMV
Enter into the system and user environment variables interface, add the NuSMV include and bin

user Variables --> new 
variable: NUSMV_LIBRARY_PATH
value : "NuSMV directory"\NuSMV-2.5.4-x86_64-w64-mingw32\share\nusmv

system Variables -->path -->edit
add new by
"NuSMV directory"\NuSMV-2.5.4-x86_64-w64-mingw32\bin
