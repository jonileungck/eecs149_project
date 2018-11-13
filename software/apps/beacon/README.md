Ultrasonic Beacon  
==============

Based on  
https://github.com/lab11/buckler/tree/master/software/apps/button_and_switch

Gives a trigger pulse to drive the Grove Ultrasonic range finder and ignore the response.  
The trigger signal goes out through pins A0, D0, SCL, and TX all at the same time.  

If SWITCH0 is set, the Buckler waits for a button press to send out a trigger pulse.  
If SWITCH0 is clear, the Buckler sends out a pulse every 100ms.  

Detection range is severely reduced when the reflector cone is installed:  

Range when reflector on both receiver and transmitter: 1m  
Range when reflector on receiver only: 2m  
Range when no reflector: >4m  
