#include "herkulex_rob.h"

//---------------------------------------------------------------------------------------------

void deverouillage_torque(void)  //débloquer les servomoteurs
{
            pc.printf("\ndeverouillage_torque\n\r");
            setTorque(1 ,TORQUE_ON ,1);setTorque(4 ,TORQUE_ON ,1);
            setTorque(2 ,TORQUE_ON ,1);setTorque(5 ,TORQUE_ON ,1);
            setTorque(3 ,TORQUE_ON ,1);setTorque(6 ,TORQUE_ON ,1);
            
            setTorque(101 ,TORQUE_ON ,1);setTorque(4 ,TORQUE_ON ,2);
            setTorque(102 ,TORQUE_ON ,1);setTorque(5 ,TORQUE_ON ,2);
            setTorque(3 ,TORQUE_ON ,2);setTorque(6 ,TORQUE_ON ,2);
            
            setTorque(1 ,TORQUE_ON ,3);setTorque(4 ,TORQUE_ON ,3);
            setTorque(2 ,TORQUE_ON ,3);setTorque(5 ,TORQUE_ON ,3);
            setTorque(3 ,TORQUE_ON ,3);setTorque(6 ,TORQUE_ON ,3);   
            
            setTorque(7 ,TORQUE_ON,4);setTorque(10 ,TORQUE_ON,4);
            
            setTorque(8 ,TORQUE_ON,4);setTorque(9  ,TORQUE_ON,4);setTorque(11 ,TORQUE_ON,4);
            
}

void verouillage_torque(void)  //débloquer les servomoteurs
{
            pc.printf("\ndeverouillage_torque\n\r");
            setTorque(1 ,BREAK_ON ,1);setTorque(4 ,BREAK_ON ,1);
            setTorque(2 ,BREAK_ON ,1);setTorque(5 ,BREAK_ON ,1);
            setTorque(3 ,BREAK_ON ,1);setTorque(6 ,BREAK_ON ,1);
            
            setTorque(1 ,BREAK_ON ,2);setTorque(4 ,BREAK_ON ,2);
            setTorque(2 ,BREAK_ON ,2);setTorque(5 ,BREAK_ON ,2);
            setTorque(3 ,BREAK_ON ,2);setTorque(6 ,BREAK_ON ,2);
            
            setTorque(1 ,BREAK_ON ,3);setTorque(4 ,BREAK_ON ,3);
            setTorque(2 ,BREAK_ON ,3);setTorque(5 ,BREAK_ON ,3);
            setTorque(3 ,BREAK_ON ,3);setTorque(6 ,BREAK_ON ,3);
            
            setTorque(7 ,BREAK_ON,4);setTorque(10 ,BREAK_ON,4);
            
            setTorque(8 ,BREAK_ON,5);setTorque(9  ,BREAK_ON,5);setTorque(11 ,BREAK_ON,5);  
}
//peut être pas utile
/*void clear_all(void)
{
    clear(1,1);clear(4,1);
    clear(2,1);clear(5,1);
    clear(3,1);clear(6,1);
    
    clear(1,2);clear(4,2);
    clear(2,2);clear(5,2);
    clear(3,2);clear(6,2);
    
    clear(1,3);clear(4,3);
    clear(2,3);clear(5,3);
    clear(3,3);clear(6,3); 
}*/