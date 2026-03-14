#ifndef MAIN_H
#define MAIN_H
#define ROBOT_SMALL 


#include "mbed.h"
#include "Asservissement.h"
#include "ident_crac.h"
#include "Capteur.h"
#include "fonctions_herkulex.h"
#include "herkulex_rob.h"
#include "dt.h"
#include "actions_Pr.h"
#include "lecture_girouette.h"



extern CAN can;
extern char cote;
extern Serial pc;

extern char bras_choix;
extern char num_ca,msg_carre;

extern unsigned short ackFinAction;
#endif