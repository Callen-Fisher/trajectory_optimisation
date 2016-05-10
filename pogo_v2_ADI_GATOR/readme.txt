Code written by Callen Fisher
email: callenfisher5@gmail.com

Start point: MAIN.m >>to perform trajectory optimisation
             SHOW_RESULTS.m>> to show the last saved results

The code attempts to optimize a pogo stick for periodic motion. This means 
that the pogo stick must start and end in the same state. 

fmincon is used to optimize the pogo stick and lagrange dynamics was used 
to model the system. Two models were constructed, the aerial and stance 
model. The generalized coordinates for the aerial model was z and l. z is 
the height of mass 1 and l is the length of the pogo stick (the distance 
between mass1 and mass 2). The generalized coordinates for the stance phase 
was just z, as z=l when the pogo stick is in contact with the ground

Multipleshooting along with multiple phase trajectory optimisation was used 
to optimize the problem. Multipleshoot is when you take the trajectory and 
split it up into nNodes and integrate between the nodes. The optimizer must 
minimize the defects which is the difference between the end state of the 
node and the start state of the next node.  

multiple phase is when different models are used for different phases. 
Model 1 was used for the aerial phase and model 2 was used for the stance 
phase. Each phase was optimized using multipleshooting and the optimizer 
stiched the three (flight, stance, flight) phases together. 

the pogo stick started in the air and then dropped to the floor, then 
bounced up again. The objective of the optimizer was to minimize the 
applied force.