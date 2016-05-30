Code written by Callen Fisher
email: callenfisher5@gmail.com

Start point: MAIN.m >>to perform trajectory optimisation
             SHOW_RESULTS.m>> to show the last saved results

The code attempts to optimize a pogo stick for periodic motion. This means 
that the pogo stick must start and end in the same state. Using the auto gen
functions from V2 of the code, it was modified to optimise using snopt
instead of fmincon
