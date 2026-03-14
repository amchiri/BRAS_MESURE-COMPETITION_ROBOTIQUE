#ifndef CRAC_ASSERVISSEMENT
#define CRAC_ASSERVISSEMENT
#include "main.h"

void Send2Char(unsigned short id, unsigned char d1, unsigned char d2);
void Send2Short(unsigned short id, unsigned short d1, unsigned short d2);
void SendMsgCan(unsigned short id, unsigned char* data, int len);
void SendCharCan(unsigned short id, unsigned char data);
/*********************************************************************************************************/
/* FUNCTION NAME: SendRawId                                                                              */
/* DESCRIPTION  : Envoie un message sans donnée, c'est-à-dire contenant uniquement un ID, sur le bus CAN */
/*********************************************************************************************************/
void SendRawId (unsigned short id);

/*********************************************************************************************/
/* FUNCTION NAME: SendAck                                                                    */
/* DESCRIPTION  : Envoyer un acknowledge                                                     */
/*********************************************************************************************/
void SendAck(unsigned short id, unsigned short from);

/*********************************************************************************************/
/* FUNCTION NAME: GoToPosition                                                               */
/* DESCRIPTION  : Transmission CAN correspondant à un asservissement en position (x,y,theta) */
/*********************************************************************************************/

void GoToPosition (unsigned short x,unsigned short y,signed short theta,signed char sens);

/****************************************************************************************/
/* FUNCTION NAME: Rotate                                                                */
/* DESCRIPTION  : Transmission CAN correspondant à une rotation                         */
/****************************************************************************************/

void Rotate (signed short angle);

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
void GoStraight (signed short distance,unsigned char recalage, unsigned short newValue, unsigned char isEnchainement);

/********************************************************************************************/
/* FUNCTION NAME: BendRadius                                                                */
/* DESCRIPTION  : Transmission CAN correspondant à un rayon de courbure                     */
/********************************************************************************************/
void BendRadius (unsigned short rayon,signed short angle,signed char sens, unsigned char enchainement);

void SetOdometrie (unsigned short canId, unsigned short x,unsigned short y,signed short theta);

/****************************************************************************************/
/* FUNCTION NAME: setAsservissementEtat                                                 */
/* DESCRIPTION  : Activer ou désactiver l'asservissement                                */
/****************************************************************************************/
void setAsservissementEtat(unsigned char enable);

/****************************************************************************************/
/* FUNCTION NAME: SendSpeed                                                             */
/* DESCRIPTION  : Envoie un asservissement paramètre retournant à une vitesse           */
/****************************************************************************************/
void SendSpeed (unsigned short vitesse, unsigned short acceleration,unsigned short deceleration);
/****************************************************************************************/
/* FUNCTION NAME: SendSpeedDecel                                                        */
/* DESCRIPTION  : Envoie un asservissement paramètre retournant à une vitesse           */
/****************************************************************************************/
void SendSpeedDecel (unsigned short vitesse, unsigned short deceleration);

#endif