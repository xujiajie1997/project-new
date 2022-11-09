
/*******Genernal Parameters for robot********/
#define H  (0.8276/2)    // The length between wheel1 and wheel2.
#define B  0.0625    // Bias of the wheels, it is usually equal to radius
#define R  0.0625    // Radius of the wheels. 
                      //circle of the wheels is 2*pi*r=0.23561944
#define Circum (2*M_PI*R)
//Down  Roll A Steel C UP Roll B Steel D
/**************Genernal Parameters for motor******************/
#define const_motor_A 170000
#define const_motor_B 200000
#define const_motor_C 170000
#define const_motor_D 200000
#define const_motor_E 170000
#define const_motor_F 200000
#define const_motor_G 170000
#define const_motor_H 200000


#define Acc_const_A  (1*const_motor_A)                //This means if set accleration to 1*Acc_const, then the accleration is 1r/s^2=0.235619m/S^2
#define Acc_const_B  (1*const_motor_B)
#define Acc_const_C  (1*const_motor_C)
#define Acc_const_D  (1*const_motor_D)
#define Acc_const_E  (1*const_motor_E)
#define Acc_const_F  (1*const_motor_F)
#define Acc_const_G  (1*const_motor_G)
#define Acc_const_H  (1*const_motor_H)


#define Dcc_const_A  (1*const_motor_A)                //This means if set decleration to 1*Dcc_const, then the decleration is 1r/s^2=0.235619m/S^2
#define Dcc_const_B  (1*const_motor_B)
#define Dcc_const_C  (1*const_motor_C)
#define Dcc_const_D  (1*const_motor_D)
#define Dcc_const_E  (1*const_motor_E)
#define Dcc_const_F  (1*const_motor_F)
#define Dcc_const_G  (1*const_motor_G)
#define Dcc_const_H  (1*const_motor_H)


#define Vel_const_A  (1*const_motor_A)                //This means if set velocity to 1*Vel_const, then the velocity is 1 turns per second.
#define Vel_const_B  (1*const_motor_B)
#define Vel_const_C  (1*const_motor_C)
#define Vel_const_D  (1*const_motor_D)
#define Vel_const_E  (1*const_motor_E)
#define Vel_const_F  (1*const_motor_F)
#define Vel_const_G  (1*const_motor_G)
#define Vel_const_H  (1*const_motor_H)


#define Pos_const_A  (1*const_motor_A)               //This means if set relative position to 1*Pos_const, then the system position change is 1 turn.
#define Pos_const_B  (1*const_motor_B)
#define Pos_const_C  (1*const_motor_C)
#define Pos_const_D  (1*const_motor_D)
#define Pos_const_E  (1*const_motor_E)
#define Pos_const_F  (1*const_motor_F)
#define Pos_const_G  (1*const_motor_G)
#define Pos_const_H  (1*const_motor_H)


#define Max_vel_turns   6
#define Delay_stop_time 1000    //5ms

#define loop_freq 300
#define AccT   6
#define DccT   6
#define XYW_conf 0.3
#define max_velcoff   1  //max velocity 0.7m/s
