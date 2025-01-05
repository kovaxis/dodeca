# Dodeca - A kitchen tabletop timer shaped like a dodecahedron

The timer is a 3D-printed dodecahedron with 10 timer-faces and 2 faces with screens:

![image](https://github.com/user-attachments/assets/61a4b356-40e4-4ecf-801b-d620075045c8)

When rotated such that a time-labelled shape faces up, the timer starts counting down:

![image](https://github.com/user-attachments/assets/c671a1c0-c08b-4c3f-9c59-d3aed39e4de4)

The timer has a battery and a Micro-USB port, that allows charging:

![image](https://github.com/user-attachments/assets/573bd384-c25b-4ab1-b211-22eb64c78595)

Inside, an Arduino connected to an accelerometer does the heavy lifting. The Arduino stays most of the time in heavy sleep, waiting for the accelerometer to wake it up through interrupts:

![image](https://github.com/user-attachments/assets/46bb1b33-d06c-4545-8e27-b198259105ff)

The face opposite to the Arduino has a button for forceful shutdown (for transport purposes), the USB port and the buzzer:

![image](https://github.com/user-attachments/assets/4cb43164-e2ca-412a-8686-f286d85e2a46)

The source code for the Arduino program is available on this repo. The 3D shape of the encasing is also available under the `tools` directory, as well as the scripts to translate MIDI and PNGs to an embedded friendly format ready to be flashed into the Arduino.
