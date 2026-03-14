#include "Asservissement.h"

/*********************************************************************************************************/
/* FUNCTION NAME: SendRawId                                                                              */
/* DESCRIPTION  : Envoie un message sans donnée, c'est-à-dire contenant uniquement un ID, sur le bus CAN */
/*********************************************************************************************************/
void SendRawId (unsigned short id)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=id;
    msgTx.len=0;
    can.write(msgTx);
    wait_us(200);
}

/*********************************************************************************************/
/* FUNCTION NAME: SendAck                                                                    */
/* DESCRIPTION  : Envoyer un acknowledge                                                     */
/*********************************************************************************************/
void SendAck(unsigned short id, unsigned short from)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=id;
    msgTx.len=2;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // from sur 2 octets
    msgTx.data[0]=(unsigned char)from;
    msgTx.data[1]=(unsigned char)(from>>8);

    can.write(msgTx);
}


void Send2Short(unsigned short id, unsigned short d1, unsigned short d2)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=id;
    msgTx.len=4;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // from sur 2 octets
    msgTx.data[0]=(unsigned char)d1;
    msgTx.data[1]=(unsigned char)(d1>>8);
    msgTx.data[2]=(unsigned char)d2;
    msgTx.data[3]=(unsigned char)(d2>>8);

    can.write(msgTx);
}

void Send2Char(unsigned short id, unsigned char d1, unsigned char d2)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=id;
    msgTx.len=2;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // from sur 2 octets
    msgTx.data[0]=(unsigned char)d1;
    msgTx.data[1]=(unsigned char)d2;

    can.write(msgTx);
}

void SendMsgCan(unsigned short id, unsigned char* data, int len)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=id;
    msgTx.len=len;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // from sur 2 octets
    for(int i = 0; i<len; i++)
    {
        msgTx.data[i]=data[i];
    }

    can.write(msgTx);
}
void SendCharCan(unsigned short id, unsigned char data)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=id;
    msgTx.len=1;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    msgTx.data[0]=data;
    
    can.write(msgTx);
}



/*********************************************************************************************/
/* FUNCTION NAME: GoToPosition                                                               */
/* DESCRIPTION  : Transmission CAN correspondant à un asservissement en position (x,y,theta) */
/*********************************************************************************************/
void GoToPosition (unsigned short x,unsigned short y,signed short theta,signed char sens)
{
    //id_to_expect=ACK_CONSIGNE;

    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_XYT; // tx nouvelle position en (x,y,theta)
    msgTx.len=7;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // x sur 2 octets
    msgTx.data[0]=(unsigned char)x;
    msgTx.data[1]=(unsigned char)(x>>8);
    // y sur 2 octets
    msgTx.data[2]=(unsigned char)y;
    msgTx.data[3]=(unsigned char)(y>>8);
    // theta signé sur 2 octets
    msgTx.data[4]=(unsigned char)theta;
    msgTx.data[5]=(unsigned char)(theta>>8);
    msgTx.data[6]=sens;

    can.write(msgTx);
}

/****************************************************************************************/
/* FUNCTION NAME: Rotate                                                                */
/* DESCRIPTION  : Transmission CAN correspondant à une rotation                         */
/****************************************************************************************/
void Rotate (signed short angle)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_ROTATION;  // Tx rotation autour du centre du robot
    msgTx.len=2;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    //  Angle signé sur 2 octets
    msgTx.data[0]=(unsigned char)angle;
    msgTx.data[1]=(unsigned char)(angle>>8);

    can.write(msgTx);
}


/*********************************************************************************************/
/* FUNCTION NAME: GoStraight                                                                 */
/* DESCRIPTION  : Transmission CAN correspondant à une ligne droite, avec ou sans recalage   */
/*  recalage : 0 => pas de recalage                                                          */
/*             1 => recalage en X                                                            */
/*             2 => Recalage en Y                                                            */
/*  newValue : Uniquement en cas de recalage, indique la nouvelle valeur de l'odo            */
/*  isEnchainement : Indique si il faut executer l'instruction en enchainement               */
/*                   0 => non                                                                */
/*                   1 => oui                                                                */
/*                   2 => dernière instruction de l'enchainement                             */
/*********************************************************************************************/
void GoStraight (signed short distance,unsigned char recalage, unsigned short newValue, unsigned char isEnchainement)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_RECALAGE;
    msgTx.len=6;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // x sur 2 octets
    msgTx.data[0]=(unsigned char)distance;
    msgTx.data[1]=(unsigned char)(distance>>8);
    //Recalage sur 1 octet
    msgTx.data[2]=recalage;
    //Valeur du recalage sur 2 octets
    msgTx.data[3]=(unsigned char)newValue;
    msgTx.data[4]=(unsigned char)(newValue>>8);
    //Enchainement sur 1 octet
    msgTx.data[5]=isEnchainement;

    can.write(msgTx);
    //wait_ms(500);
}

/********************************************************************************************/
/* FUNCTION NAME: BendRadius                                                                */
/* DESCRIPTION  : Transmission CAN correspondant à un rayon de courbure                     */
/********************************************************************************************/
void BendRadius (unsigned short rayon,signed short angle,signed char sens, unsigned char enchainement)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_COURBURE; // tx asservissement rayon de courbure
    msgTx.len=6;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    // Rayon sur 2 octets
    msgTx.data[0]=(unsigned char)rayon;
    msgTx.data[1]=(unsigned char)(rayon>>8);
    // Angle signé sur 2 octets
    msgTx.data[2]=(unsigned char)angle;
    msgTx.data[3]=(unsigned char)(angle>>8);
    // Sens signé sur 1 octet
    msgTx.data[4]=sens;
    // Enchainement sur 1 octet
    msgTx.data[5]=enchainement;

    can.write(msgTx);
}

void SetOdometrie (unsigned short canId, unsigned short x,unsigned short y,signed short theta)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=canId;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    msgTx.len=6;

        // x sur 2 octets
    msgTx.data[0]=(unsigned char)x;
    msgTx.data[1]=(unsigned char)(x>>8);
    // y sur 2 octets
    msgTx.data[2]=(unsigned char)y;
    msgTx.data[3]=(unsigned char)(y>>8);
    // theta signé sur 2 octets
    msgTx.data[4]=(unsigned char)theta;
    msgTx.data[5]=(unsigned char)(theta>>8);

    can.write(msgTx);
}

/****************************************************************************************/
/* FUNCTION NAME: setAsservissementEtat                                                 */
/* DESCRIPTION  : Activer ou désactiver l'asservissement                                */
/****************************************************************************************/
void setAsservissementEtat(unsigned char enable)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_ENABLE;  // Tx rotation autour du centre du robot
    msgTx.len=1;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    //  Angle signé sur 2 octets
    msgTx.data[0]=(unsigned char)((enable==0)?0:1);

    can.write(msgTx);
}


/****************************************************************************************/
/* FUNCTION NAME: SendSpeed                                                             */
/* DESCRIPTION  : Envoie un asservissement paramètre retournant à une vitesse           */
/****************************************************************************************/
void SendSpeed (unsigned short vitesse, unsigned short acceleration,unsigned short deceleration)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_CONFIG;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    msgTx.len=8;
    msgTx.data[0]=(unsigned char)(vitesse&0x00FF);
    msgTx.data[1]=(unsigned char)((vitesse&0xFF00)>>8);
    
    msgTx.data[2]=(unsigned char)(acceleration&0x00FF);
    msgTx.data[3]=(unsigned char)((acceleration&0xFF00)>>8);
    
    msgTx.data[4]=(unsigned char)(deceleration&0x00FF);
    msgTx.data[5]=(unsigned char)((deceleration&0xFF00)>>8);
    
    msgTx.data[6]=(unsigned char)(acceleration&0x00FF);//cloto
    msgTx.data[7]=(unsigned char)((acceleration&0xFF00)>>8);//cloto

    can.write(msgTx);
        
}

/****************************************************************************************/
/* FUNCTION NAME: SendSpeedDecel                                                        */
/* DESCRIPTION  : Envoie un asservissement paramètre retournant à une vitesse           */
/****************************************************************************************/
 /*
void SendSpeedDecel (unsigned short vitesse, unsigned short deceleration)
{
    CANMessage msgTx=CANMessage();
    msgTx.id=ASSERVISSEMENT_CONFIG_DECEL;
    msgTx.format=CANStandard;
    msgTx.type=CANData;
    msgTx.len=4;
    msgTx.data[0]=(unsigned char)(vitesse&0x00FF);
    msgTx.data[1]=(unsigned char)((vitesse&0xFF00)>>8);
    msgTx.data[2]=(unsigned char)(deceleration&0x00FF);
    msgTx.data[3]=(unsigned char)((deceleration&0xFF00)>>8);
 
    can.write(msgTx);
        
}*/
   
   