/****************************************************************/
#include "parameters_mobilemanipulator.h"
#define Max_Acc (100*(double)Circum)                      //=23.5619m/s^2
#define Max_Vel (6*(double)Circum)                        //=1.38m/s
#define Max_jerk (330*(double)Circum)

#define Scurve_Acc  (0.5*(double)Circum)
#define Scurve_Vel  (0.03*(double)Circum)
#define Scurve_Jerk (1.5*(double)Circum)
#define pub_velfreq  (loop_freq)

#define deltaT  (1/(double)pub_velfreq)
#define Acc_Tm   ((double)Scurve_AccTime/3)
#define Dcc_Tm   ((double)Scurve_DccTime/3)

#define absolute (0.000001)
