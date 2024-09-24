
#ifndef FSM_H
#define FSM_H

#include "stm32f4xx_hal.h"

#define INSPECTION -8.0
#define Ready 0x10

#define Remote_ZONE1 0x11
#define Remote_ZONE2 0x12

#define ZONE_1   0x01
#define ZONE_2   0x02


#define Insp 0x01
#define Grab 0x02
#define Put 0x03

#define SHOOT_Action  0
#define	Left_Up  	  1
#define	Right_Up	  2
#define	Left_Down	  3	
#define	Right_Down    4
	
#define	Left_Turn     5
#define	Right_Turn    6

#define OK 	  0x01
#define ERROR 0

typedef struct
{
	unsigned char Action_State;
	unsigned char Move_Start;
	unsigned char Inspection_Flag;
	unsigned char Send_Inspection;
}MOVE_STATE;

extern MOVE_STATE Move_State;
extern int Last_SWA;
extern int Last_SWB;
extern int Last_SWC;
extern int Last_SWD;
extern unsigned short last_mode;
extern int last_mid;
extern unsigned short Hand_Position;
extern uint8_t ZONE_Choice;

void move(void);
void shoot_test(void);
void move_test(void);
void Move_Red(void);
void ActionReceive_FSM(void);
void Remote_FSM(void);
void Move_FSM(void);
#endif /* FSM_H */

