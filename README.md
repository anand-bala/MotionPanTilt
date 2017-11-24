# Motion Pan-Tilt

The Hand Controlled Pan-Tilt Stand uses motion sensors attached to the hand to obtain the orientation of the hand and use this to drive the servo motors that control the pan and tilt of the stand. It uses an Arduino Uno for all the computations, a 6DOF motion sensor and two servo motors (for pan and tilt).

It is designed such that there is a motion sensor attached to the hand from which we can obtain the orientation of the hand in terms of angles along x-, y-, and z-axes (Pitch, Roll and Yaw) and these angles can in turn be used to drive the angle of the servos on the stand. While it uses a single Arduino Uno for the computations, the components can be split up using wireless technology, as the only things that need to be communicated to the servos are the angles.

![alt text](https://raw.githubusercontent.com/anand-bala/MotionPanTilt/master/TermProject_bb.jpg)
