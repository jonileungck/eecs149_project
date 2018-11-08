Ultrasonic Receiver  
==============

Wait for an ultrasonic detector interrupt, record the time that the interrupt happens, then log the time on terminal.  

Detection range is severely reduced when the reflector cone is installed:  

Range when reflector on both receiver and transmitter: 1m  
Range when reflector on receiver only: 2m  
Range when no reflector: >4m  

must comment out nrfx_timer.c from Board.mk, since we redefined timer IRQ handler.  
