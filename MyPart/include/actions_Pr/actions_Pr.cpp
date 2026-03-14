#include "actions_Pr.h"

Timer timeout;
//variables//
char aut_bras_av_at = 0,   aut_bras_av_re = 0;
char aut_bras_av_1_at = 0, aut_bras_av_1_re = 0;
char aut_bras_av_2_at = 0, aut_bras_av_2_re = 0;
char aut_bras_av_3_at = 0, aut_bras_av_3_re = 0;
char aut_manche_haut = 0,  aut_manche_bas = 0, aut_manche_moy = 0 ;
char aut_bras_av_prepa = 0,   aut_bras_av_pose = 0;
char aut_bras_av_1_prepa = 0, aut_bras_av_1_pose = 0;
char aut_bras_av_2_prepa = 0, aut_bras_av_2_pose = 0;
char aut_bras_av_3_prepa = 0, aut_bras_av_3_pose = 0;


//moi
int tab[7]= {9,Mycolor,9,9,9,9,9};
bool choix_color=0;
AnalogIn releve(PA_5);
bool delay=true;
int resultat;
uint8_t correction=0;
char msg_num_ca=0;

//

unsigned char var_bras_choix_1 = 0, var_bras_choix_2= 0, var_bras_choix_3= 0, var_bras_manche = 0;
char vitesse_bras = 35; //50

//define bras gauche et droit
uint8_t servos_bras_avant[6] = {RLED_ON, 1, GLED_ON, 2, BLED_ON, 3};
uint8_t servos_bras_arriere[6] = {RLED_ON, 4, GLED_ON, 5, BLED_ON, 6};


//position des bras
uint16_t pos_initial_1[3] = {600,200,200};
uint16_t pos_prepa_attraper_1[3] = {512,220,200};
uint16_t pos_revenir_1[3] = {600,200,70};
uint16_t pos_prepa_relacher_1[3] = {512,220,70};

uint16_t pos_initial_2[3] = {760,130,200};
uint16_t pos_prepa_relacher_2[3] = {240,512,70};
uint16_t pos_prepa_attraper_2[3] = {300,630,200}; //220,...,...
uint16_t pos_revenir_2[3] = {760,150,70};

uint16_t pos_prepa_attraper_3[3] = {320,430,200};

uint16_t pos_attraper[3] = {270,430,70};
uint16_t pos_relacher[3] = {240,512,200};

void gabarit_robot_droit(void)
{
    pc.printf("\ngabarit robot avant\n\n");
    positionControl_Mul_ensemble_complex(3, 50, servos_bras_avant, pos_initial_2, 1);
    positionControl_Mul_ensemble_complex(3, 50, servos_bras_avant, pos_initial_2, 3);
    positionControl_Mul_ensemble_complex(3, 50, servos_bras_avant, pos_initial_2, 2);
    verification();
}

void gabarit_robot_gauche(void)
{
    pc.printf("\ngabarit robot arriere\n\n");
    positionControl_Mul_ensemble_complex(3, 50, servos_bras_arriere, pos_initial_2, 1);
    positionControl_Mul_ensemble_complex(3, 50, servos_bras_arriere, pos_initial_2, 2);
    positionControl_Mul_ensemble_complex(3, 50, servos_bras_arriere, pos_initial_2, 3);
    verification();
}

void gabarit_robot_manche(void)
{
    positionControl(7,220,80,RLED_ON,4);
    positionControl(8,240,80,RLED_ON,4);
    positionControl(11,780,80,RLED_ON,4); //pour rajouter le drapeau changer le baudrate !!
    verification();
}

///////////////////////////////////////////////////fonction de test////////////////////////////////////////////////
void test_BRAS_1(void)
{
    positionControl(7,410,20, RLED_ON,4);
    verification();
    positionControl(8,410,20, RLED_ON,4);
    //positionControl(11,230,80,0,4);
    verification();
    wait(1);
    positionControl(7,230,20,RLED_ON,4);
    verification();
    positionControl(8,230,20,RLED_ON,4);
    //positionControl(11,780,80,0,4);
    verification();
}

void test_BRAS_2(void)
{
    verification();
}

void test_BRAS_3(void)
{
    verification();
}

void test_BRAS_4(void)
{
    verification();
}

///////////////////////////////////////////////SELECTION ATTRAPER BRAS//////////////////////////////////////////////
void selection_bras_attraper(void)
{
    if(aut_bras_av_at) {
        if(bras_choix<6) {
            aut_bras_av_1_at = aut_bras_av_at;
            automate_bras_attraper_1();
        } else if(bras_choix>5) {
            switch(bras_choix) {
                case 10:
                    aut_bras_av_2_at=aut_bras_av_at;
                    automate_bras_attraper_2();
                    break;

                case 20:
                    aut_bras_av_2_at=aut_bras_av_at;
                    automate_bras_attraper_2();
                    break;

                case 21:
                    aut_bras_av_2_at=aut_bras_av_at;
                    automate_bras_attraper_2();
                    break;

                case 210:
                    aut_bras_av_3_at=aut_bras_av_at;
                    automate_bras_attraper_3();
                    break;

                case 43:
                    aut_bras_av_2_at=aut_bras_av_at;
                    automate_bras_attraper_2();
                    break;

                case 53:
                    aut_bras_av_2_at=aut_bras_av_at;
                    automate_bras_attraper_2();
                    break;

                case 54:
                    aut_bras_av_2_at=aut_bras_av_at;
                    automate_bras_attraper_2();
                    break;

                case 66:
                    aut_bras_av_3_at=aut_bras_av_at;
                    automate_bras_attraper_3();
                    break;
            }
        }
    }
}

///////////////////////////////////////////////SELECTION RELACHER BRAS//////////////////////////////////////////////
void selection_bras_relacher(void)
{
    if(aut_bras_av_re) {
        if(bras_choix<6) {
            aut_bras_av_1_re = aut_bras_av_re;
            automate_bras_relacher_1();
        } else if(bras_choix>5) {
            switch(bras_choix) {
                case 10:
                    aut_bras_av_2_re=aut_bras_av_re;
                    automate_bras_relacher_2();
                    break;

                case 20:
                    aut_bras_av_2_re=aut_bras_av_re;
                    automate_bras_relacher_2();
                    break;

                case 21:
                    aut_bras_av_2_re=aut_bras_av_re;
                    automate_bras_relacher_2();
                    break;

                case 210:
                    aut_bras_av_3_re=aut_bras_av_re;
                    automate_bras_relacher_3();
                    break;

                case 43:
                    aut_bras_av_2_re=aut_bras_av_re;
                    automate_bras_relacher_2();
                    break;

                case 53:
                    aut_bras_av_2_re=aut_bras_av_re;
                    automate_bras_relacher_2();
                    break;

                case 54:
                    aut_bras_av_2_re=aut_bras_av_re;
                    automate_bras_relacher_2();
                    break;

                case 66:
                    aut_bras_av_3_re=aut_bras_av_re;
                    automate_bras_relacher_3();
                    break;
            }
        }
    }
}

///////////////////////////////////////////////SELECTION PREPA BRAS//////////////////////////////////////////////
void selection_bras_prepa(void)
{
    if(aut_bras_av_prepa) {
        if(bras_choix<6) {
            aut_bras_av_1_prepa = aut_bras_av_prepa;
            automate_bras_prepa_1();
        } else if(bras_choix>5) {
            switch(bras_choix) {
                case 10:
                    aut_bras_av_2_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_2();
                    break;

                case 20:
                    aut_bras_av_2_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_2();
                    break;

                case 21:
                    aut_bras_av_2_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_2();
                    break;

                case 210:
                    aut_bras_av_3_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_3();
                    break;

                case 43:
                    aut_bras_av_2_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_2();
                    break;

                case 53:
                    aut_bras_av_2_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_2();
                    break;

                case 54:
                    aut_bras_av_2_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_2();
                    break;

                case 66:
                    aut_bras_av_3_prepa=aut_bras_av_prepa;
                    automate_bras_prepa_3();
                    break;
            }
        }
    }
}

///////////////////////////////////////////////SELECTION POSE BRAS//////////////////////////////////////////////
void selection_bras_poser(void)
{
    if(aut_bras_av_pose) {
        if(bras_choix<6) {
            aut_bras_av_1_pose = aut_bras_av_pose;
            automate_bras_poser_1();
        } else if(bras_choix>5) {
            switch(bras_choix) {
                case 10:
                    aut_bras_av_2_pose=aut_bras_av_pose;
                    automate_bras_poser_2();
                    break;

                case 20:
                    aut_bras_av_2_pose=aut_bras_av_pose;
                    automate_bras_poser_2();
                    break;

                case 21:
                    aut_bras_av_2_pose=aut_bras_av_pose;
                    automate_bras_poser_2();
                    break;

                case 210:
                    aut_bras_av_3_pose=aut_bras_av_pose;
                    automate_bras_poser_3();
                    break;

                case 43:
                    aut_bras_av_2_pose=aut_bras_av_pose;
                    automate_bras_poser_2();
                    break;

                case 53:
                    aut_bras_av_2_pose=aut_bras_av_pose;
                    automate_bras_poser_2();
                    break;

                case 54:
                    aut_bras_av_2_pose=aut_bras_av_pose;
                    automate_bras_poser_2();
                    break;

                case 66:
                    aut_bras_av_3_pose=aut_bras_av_pose;
                    automate_bras_poser_3();
                    break;
            }
        }
    }
}

/**********************************************BRAS ATTRAPER*****************************************/


///////////////////////////////////////////////1 BRAS//////////////////////////////////////////////
void automate_bras_attraper_1(void)
{

    typedef enum {init, aut_pos_prepa_attraper_1, aut_pos_prepa_attraper_2, aut_pos_prepa_attraper_3, aut_pos_attraper, aut_pos_revenir_1, aut_pos_revenir_2} type_etat;
    static type_etat etat = init;
    static unsigned char sens_avant=0;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_1_at) {
                var_bras_choix_1 = bras_choix;
                sens_avant=0;
                if(var_bras_choix_1<3) {
                    sens_avant=1;
                    if(var_bras_choix_1==0)
                        var_bras_choix_1=3;
                    else if(var_bras_choix_1==1)
                        var_bras_choix_1=2;
                    else if(var_bras_choix_1==2)
                        var_bras_choix_1=1;
                }
                etat=aut_pos_prepa_attraper_1;
            }
            break;

        case aut_pos_prepa_attraper_1://envoi instruction mouvement 1
            if(sens_avant == 0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, var_bras_choix_1-2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_prepa_attraper_2;
            break;

        case aut_pos_prepa_attraper_2://envoi instruction mouvement 2
            if(sens_avant == 0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, var_bras_choix_1-2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_prepa_attraper_3;
            break;

        case aut_pos_prepa_attraper_3://envoi instruction mouvement 1
            if(sens_avant == 0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, var_bras_choix_1-2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_attraper;
            break;

        case aut_pos_attraper://envoi instruction mouvement 1
            if(sens_avant == 0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, var_bras_choix_1-2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_revenir_1;
            break;

        case aut_pos_revenir_1://envoi instruction mouvement 1
            if(sens_avant == 0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, var_bras_choix_1-2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_revenir_2;
            break;

        case aut_pos_revenir_2://envoi instruction mouvement 1
            if(sens_avant == 0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, var_bras_choix_1-2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, var_bras_choix_1);
            }
            verification();
            aut_bras_av_1_at = 0;
            aut_bras_av_at = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;

    }
}

void automate_bras_relacher_1(void)
{
    typedef enum {init, aut_pos_prepa_relacher_1, aut_pos_prepa_relacher_2, aut_pos_relacher, aut_pos_initial_1, aut_pos_initial_2} type_etat;
    static type_etat etat = init;
    static unsigned char sens_avant=0;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_1_re) {
                var_bras_choix_1 = bras_choix;
                sens_avant=0;
                if(var_bras_choix_1<3) {
                    sens_avant=1;
                    if(var_bras_choix_1==0)
                        var_bras_choix_1=3;
                    else if(var_bras_choix_1==1)
                        var_bras_choix_1=2;
                    else if(var_bras_choix_1==2)
                        var_bras_choix_1=1;
                }
                etat=aut_pos_prepa_relacher_1;
            }
            break;

        case aut_pos_prepa_relacher_1://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_prepa_relacher_2;
            break;

        case aut_pos_prepa_relacher_2://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_relacher;
            break;

        case aut_pos_relacher://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_initial_1;
            break;

        case aut_pos_initial_1://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_initial_2;
            break;

        case aut_pos_initial_2://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, var_bras_choix_1);
            }
            verification();
            aut_bras_av_1_re = 0;
            aut_bras_av_re = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}
//////////////////////////////////////////////////2 bras//////////////////////////////////////////////////////
void automate_bras_attraper_2(void)
{

    typedef enum {init, aut_pos_prepa_attraper_1, aut_pos_prepa_attraper_2, aut_pos_prepa_attraper_3, aut_pos_attraper, aut_pos_revenir_1, aut_pos_revenir_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_2_at) {
                var_bras_choix_2 = bras_choix;
                etat=aut_pos_prepa_attraper_1;
            }
            break;

        case aut_pos_prepa_attraper_1://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 3);
            }
            verification();
            etat = aut_pos_prepa_attraper_2;
            break;

        case aut_pos_prepa_attraper_2://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 3);
            }
            verification();
            etat = aut_pos_prepa_attraper_3;
            break;

        case aut_pos_prepa_attraper_3://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 3);
            }
            verification();
            etat = aut_pos_attraper;
            break;

        case aut_pos_attraper://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 3);
            }
            verification();
            etat = aut_pos_revenir_1;
            break;

        case aut_pos_revenir_1://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 3);
            }
            verification();
            etat = aut_pos_revenir_2;
            break;

        case aut_pos_revenir_2://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 3);
            }
            verification();
            aut_bras_av_2_at = 0;
            aut_bras_av_at = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}

void automate_bras_relacher_2(void)
{

    typedef enum {init, aut_pos_prepa_relacher_1, aut_pos_prepa_relacher_2, aut_pos_relacher, aut_pos_initial_1, aut_pos_initial_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_2_re) {
                var_bras_choix_2 = bras_choix;
                etat=aut_pos_prepa_relacher_1;
            }
            break;

        case aut_pos_prepa_relacher_1://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 3);
            }
            verification();
            etat = aut_pos_prepa_relacher_2;
            break;

        case aut_pos_prepa_relacher_2://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 3);
            }
            verification();
            etat = aut_pos_relacher;
            break;

        case aut_pos_relacher://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 3);
            }
            verification();
            etat = aut_pos_initial_1;
            break;

        case aut_pos_initial_1://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 3);
            }
            verification();
            etat = aut_pos_initial_2;
            break;

        case aut_pos_initial_2://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 3);
            }
            verification();
            aut_bras_av_2_re = 0;
            aut_bras_av_re = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}
////////////////////////////////////////////////3 BRAS/////////////////////////////////////////////////////
void automate_bras_attraper_3(void)
{

    typedef enum {init, aut_pos_prepa_attraper_1, aut_pos_prepa_attraper_2, aut_pos_prepa_attraper_3, aut_pos_attraper, aut_pos_revenir_1, aut_pos_revenir_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_3_at) {
                var_bras_choix_3 = bras_choix;
                etat=aut_pos_prepa_attraper_1;
            }
            break;

        case aut_pos_prepa_attraper_1://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_1, 2);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_1, 3);
//               verification_3_bras(0);
                //verification();
            }
            verification();
            etat = aut_pos_prepa_attraper_2;
            break;

        case aut_pos_prepa_attraper_2://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_2, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_2, 3);
                verification_3_bras(0);
                //verification();
            }
            etat = aut_pos_prepa_attraper_3;
            break;

        case aut_pos_prepa_attraper_3://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_attraper_3, 2);
//                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_attraper_3, 3);
//                verification_3_bras(0);
                //verification();
            }
            verification();
            etat = aut_pos_attraper;
            break;

        case aut_pos_attraper://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_attraper, 2);
//                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_attraper, 3);
//                verification_3_bras(0);
                //verification();
            }
            verification();
            etat = aut_pos_revenir_1;
            break;

        case aut_pos_revenir_1://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_1, 2);
                //verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_1, 3);
                //verification_3_bras(0);
                //verification();
            }
            etat = aut_pos_revenir_2;
            break;

        case aut_pos_revenir_2://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_revenir_2, 2);
                //verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_revenir_2, 3);
                //verification_3_bras(0);
                //verification();
            }
            verification();
            aut_bras_av_3_at = 0;
            aut_bras_av_at = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}

void automate_bras_relacher_3(void)
{

    typedef enum {init, aut_pos_prepa_relacher_1, aut_pos_prepa_relacher_2, aut_pos_relacher, aut_pos_initial_1, aut_pos_initial_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_3_re) {
                etat=aut_pos_prepa_relacher_1;
            }
            var_bras_choix_3 = bras_choix;
            break;

        case aut_pos_prepa_relacher_1://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 3);
                verification_3_bras(0);
            }
            etat = aut_pos_prepa_relacher_2;
            break;

        case aut_pos_prepa_relacher_2://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 3);
                verification_3_bras(0);
            }
            etat = aut_pos_relacher;
            break;

        case aut_pos_relacher://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 2);
                //verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 3);
                //verification_3_bras(0);
            }
            etat = aut_pos_initial_1;
            break;

        case aut_pos_initial_1://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 2);
                //verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 3);
                //verification_3_bras(0);
            }
            etat = aut_pos_initial_2;
            break;

        case aut_pos_initial_2://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 3);
                verification_3_bras(0);
            }
            aut_bras_av_3_re = 0;
            aut_bras_av_re = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}

//////////////////////////////////AUTOMATE MANCHE A AIR & Drapeau/////////////////////////////////////
void automate_manche_air_haut(void)
{
    if(aut_manche_haut) {           // 0 = Manche à aire droite          1 = manche à aire gauche
        var_bras_manche = bras_choix ;
        if(var_bras_manche == 1) {
            positionControl(7,220,30,RLED_ON,4);
            verification();
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
        }

        else if (var_bras_manche == 0) {
            positionControl(8,240,30,RLED_ON,4);
            verification();
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
        }
        aut_manche_haut = 0 ;
    }
}

void automate_manche_air_bas(void)
{
    if(aut_manche_bas) {           // 0 = manche à aire droite          1 = manche à aire gauche
        var_bras_manche = bras_choix ;
        if(var_bras_manche == 1) {
            positionControl(7,512,30,RLED_ON,4);
            verification();
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
        }

        else if (var_bras_manche == 0) {
            positionControl(8,512,30,RLED_ON,4);
            verification();
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
        }
        aut_manche_bas = 0 ;
    }
}

void automate_manche_air_moy(void)
{
    if(aut_manche_moy) {           // 0 = manche à aire droite          1 = manche à aire gauche
        var_bras_manche = bras_choix ;
        if(var_bras_manche == 1) {
            positionControl(7,460,30,RLED_ON,4);
            verification();
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
        }

        else if (var_bras_manche == 0) {
            positionControl(8,460,30,RLED_ON,4);
            verification();
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
        }
        aut_manche_moy = 0 ;
    }
}

void pavilon_deploye(void)
{
    positionControl(11,230,80,RLED_ON,4);
    verification() ;
    SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
}

/**********************************************BRAS PREPA*****************************************/

///////////////////////////////////////////////1 BRAS//////////////////////////////////////////////
void automate_bras_prepa_1(void)
{
    typedef enum {init, aut_pos_prepa_relacher_1, aut_pos_prepa_relacher_2} type_etat;
    static type_etat etat = init;
    static unsigned char sens_avant=0;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_1_prepa) {
                var_bras_choix_1 = bras_choix;
                sens_avant=0;
                if(var_bras_choix_1<3) {
                    sens_avant=1;
                    if(var_bras_choix_1==0)
                        var_bras_choix_1=3;
                    else if(var_bras_choix_1==1)
                        var_bras_choix_1=2;
                    else if(var_bras_choix_1==2)
                        var_bras_choix_1=1;
                }
                etat=aut_pos_prepa_relacher_1;
            }
            break;

        case aut_pos_prepa_relacher_1://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_prepa_relacher_2;
            break;

        case aut_pos_prepa_relacher_2://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, var_bras_choix_1);
            }
            verification();

            aut_bras_av_1_prepa = 0;
            aut_bras_av_prepa = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}
//////////////////////////////////////////////////2 bras//////////////////////////////////////////////////////
void automate_bras_prepa_2(void)
{

    typedef enum {init, aut_pos_prepa_relacher_1, aut_pos_prepa_relacher_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_2_prepa) {
                var_bras_choix_2 = bras_choix;
                etat=aut_pos_prepa_relacher_1;
            }
            break;

        case aut_pos_prepa_relacher_1://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 3);
            }
            verification();
            etat = aut_pos_prepa_relacher_2;
            break;

        case aut_pos_prepa_relacher_2://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 3);
            }
            verification();
            aut_bras_av_2_prepa = 0;
            aut_bras_av_prepa = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}
////////////////////////////////////////////////3 BRAS/////////////////////////////////////////////////////
void automate_bras_prepa_3(void)
{

    typedef enum {init, aut_pos_prepa_relacher_1, aut_pos_prepa_relacher_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_3_prepa) {
                etat=aut_pos_prepa_relacher_1;
            }
            var_bras_choix_3 = bras_choix;
            break;

        case aut_pos_prepa_relacher_1://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_1, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_1, 3);
                verification_3_bras(0);
            }
            etat = aut_pos_prepa_relacher_2;
            break;

        case aut_pos_prepa_relacher_2://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_prepa_relacher_2, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_prepa_relacher_2, 3);
                verification_3_bras(0);
            }

            aut_bras_av_1_prepa = 0;
            aut_bras_av_prepa = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}

/**********************************************BRAS POSER*****************************************/

///////////////////////////////////////////////1 BRAS//////////////////////////////////////////////
void automate_bras_poser_1(void)
{
    typedef enum {init, aut_pos_relacher, aut_pos_initial_1, aut_pos_initial_2} type_etat;
    static type_etat etat = init;
    static unsigned char sens_avant=0;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_1_pose) {
                var_bras_choix_1 = bras_choix;
                sens_avant=0;
                if(var_bras_choix_1<3) {
                    sens_avant=1;
                    if(var_bras_choix_1==0)
                        var_bras_choix_1=3;
                    else if(var_bras_choix_1==1)
                        var_bras_choix_1=2;
                    else if(var_bras_choix_1==2)
                        var_bras_choix_1=1;
                }
                etat=aut_pos_relacher;
            }
            break;

        case aut_pos_relacher://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_initial_1;
            break;

        case aut_pos_initial_1://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, var_bras_choix_1);
            }
            verification();
            etat = aut_pos_initial_2;
            break;

        case aut_pos_initial_2://envoi instruction
            if(sens_avant==0) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, var_bras_choix_1 - 2);
            } else {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, var_bras_choix_1);
            }
            verification();
            aut_bras_av_1_pose = 0;
            aut_bras_av_pose = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}

//////////////////////////////////////////////////2 bras//////////////////////////////////////////////////////
void automate_bras_poser_2(void)
{

    typedef enum {init, aut_pos_relacher, aut_pos_initial_1, aut_pos_initial_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_2_pose) {
                var_bras_choix_2 = bras_choix;
                etat=aut_pos_relacher;
            }
            break;

        case aut_pos_relacher://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 3);
            }
            verification();
            etat = aut_pos_initial_1;
            break;

        case aut_pos_initial_1://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 3);
            }
            verification();
            etat = aut_pos_initial_2;
            break;

        case aut_pos_initial_2://envoi instruction
            if(var_bras_choix_2 == 21) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 2);
            } else if(var_bras_choix_2 == 20) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 3);
            } else if(var_bras_choix_2 == 10) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 3);
            } else if(var_bras_choix_2 == 43) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 2);
            } else if(var_bras_choix_2 == 53) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 3);
            } else if(var_bras_choix_2 == 54) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 3);
            }
            verification();
            aut_bras_av_2_pose = 0;
            aut_bras_av_pose = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}
int mesure_resistor(float vpot,bool color) // definition de la fontion qui mesure les resistances
{
    float sample = vpot;
    if ((sample > 0.8f) && (sample < 0.87f)) { // marge d'ereur  +-5% de ce que qu'on mesure de la resistance bleu
        if (color == 1) {
            pc.printf("test non ok \n");
            return Enemy_color;

        } else {
            pc.printf("test non ok1 \n");
            return Mycolor;


        }
    } else if ((sample > 0.37f) && (sample < 0.42f)) { // marge d'ereur +-5% de ce que qu'on mesure de la resistance rouge
        pc.printf("test ok0 \n");
        return rouge;

    } else if ((sample > 0.70f) && (sample < 0.78f)) { // marge d'ereur +-5% de ce que qu'on mesure de la resistance jaune
        if (color == 1) {
            pc.printf("test non ok2 \n");
            return Mycolor;

        } else {
            pc.printf("test non ok3 \n");
            return Enemy_color;

        }

    }
    pc.printf("noope");
    pc.printf("   %f",sample);
    return 9;

}

////////////////////////////////////////////////3 BRAS/////////////////////////////////////////////////////
void automate_bras_poser_3(void)
{

    typedef enum {init, aut_pos_relacher, aut_pos_initial_1, aut_pos_initial_2} type_etat;
    static type_etat etat = init;

    switch(etat) {
        case init: //attente d'initialisation
            if(aut_bras_av_3_pose) {
                etat=aut_pos_relacher;
            }
            var_bras_choix_3 = bras_choix;
            break;

        case aut_pos_relacher://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_relacher, 2);
                //verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_relacher, 3);
                //verification_3_bras(0);
            }
            etat = aut_pos_initial_1;
            break;

        case aut_pos_initial_1://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_1, 2);
                //verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_1, 3);
                //verification_3_bras(0);
            }
            etat = aut_pos_initial_2;
            break;

        case aut_pos_initial_2://envoi instruction
            if(var_bras_choix_3 == 210) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 3);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_avant, pos_initial_2, 2);
                verification_3_bras(1);
            } else if(var_bras_choix_3 == 66) {
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 1);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 2);
                positionControl_Mul_ensemble_complex(3, vitesse_bras, servos_bras_arriere, pos_initial_2, 3);
                verification_3_bras(0);
            }
            aut_bras_av_3_pose = 0;
            aut_bras_av_pose = 0 ;
            SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
            etat = init;
            break;
    }
}

int traitement()
{
    typedef enum {init,ranger,mesure, bascule, pretest} type_etat;
    static type_etat etat = init;
    switch(etat) {
        case init:
            if(msg_carre==1) {
                etat=ranger;
                msg_carre=0;
            } else if(msg_carre==2) {
                etat=mesure;
                msg_carre=0;
                delay=true;
            } else if(msg_carre==3) {

                etat=pretest;
                msg_carre=0;
            }

            break;

        case ranger:
        pc.printf("RANGER  ");
            positionControl(ID_MILLIEU+choix_color, 800, PLAYTIME, RLED_ON, 1);
            positionControl(ID_HAUT+choix_color, 512, PLAYTIME, RLED_ON, 1);
            if(msg_carre==2) {
                etat=mesure;
                msg_carre=0;
                delay=true;
            } else if(msg_carre==3) {

                etat=pretest;
                msg_carre=0;
            }

            break;


        case mesure:
            if(tab[num_ca]==9) {
                positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);
                positionControl(ID_MILLIEU+choix_color, 602, 40, GLED_ON, 1);
                while(delay ) {
                    resultat=mesure_resistor(releve,choix_color);
                    if(resultat==0) {
                        delay=false;
                    }
                    if(resultat==1) {
                        delay=false;
                    }
                    if(resultat==2) {
                        delay=false;
                    }
                    if(correction==30) {
                        pc.printf("eror");
                        correction=0;
                        etat=pretest;
                        break;
                    } else {
                        correction=correction+5;
                        positionControl(ID_MILLIEU, 602-correction, 40, GLED_ON, 1); // on positionne la base 565
                        pc.printf("je suis la");
                    }
                }

                tab[num_ca]=resultat;
                algo_carre (num_ca);



            }
            if(tab[num_ca]==rouge) {
                etat=pretest;
                SendCharCan(0x35,tab[num_ca]);
            } else if(tab[num_ca]==Mycolor) {
                etat=bascule;
                SendCharCan(0x35,tab[num_ca]);
            } else if(tab[num_ca]==Enemy_color) {
                etat=pretest;
                SendCharCan(0x35,tab[num_ca]);
            }
            break;


        case bascule:

            positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);
            positionControl(ID_MILLIEU+choix_color, 602, 40, GLED_ON, 1);
            positionControl(ID_HAUT+choix_color, 119, 20, GLED_ON, 1);
            positionControl(ID_HAUT+choix_color, 612, 20, GLED_ON, 1);
            positionControl(ID_MILLIEU+choix_color, 450, 20, GLED_ON, 1);
            positionControl(ID_MILLIEU+choix_color, 700, 20, GLED_ON, 1);
            positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);


            etat=pretest;//peut etre a changer

            break;

        case pretest:
            positionControl(ID_MILLIEU+choix_color, 700, 40, GLED_ON, 1); // on positionne la base 770
            positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);

            if(msg_carre==2) {
                etat=mesure;
                delay=true;
                msg_carre=0;
            } else if(msg_carre==1) {
                etat=ranger;
                msg_carre=0;
            }

            break;
    }
    return 0;
}
void set_color(bool color)
{
    choix_color=color;
}
void BF_test_mesure(bool BJ)
{
    positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);
    positionControl(ID_MILLIEU+choix_color, 602, 40, GLED_ON, 1);
    if(mesure_resistor(releve,BJ)==1) {
        positionControl(ID_HAUT+choix_color, 119, 20, GLED_ON, 1);
        positionControl(ID_HAUT+choix_color, 612, 20, GLED_ON, 1);
        positionControl(ID_MILLIEU+choix_color, 450, 20, GLED_ON, 1);
        positionControl(ID_MILLIEU+choix_color, 700, 20, GLED_ON, 1);
        positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);
        SendCharCan(0x35,0x01);

    } else if(mesure_resistor(releve,BJ)==0) {
        positionControl(ID_MILLIEU+choix_color, 700, 40, GLED_ON, 1);
        positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);
        SendCharCan(0x35,0x00);


    } else if(mesure_resistor(releve,BJ)==2) {
        positionControl(ID_MILLIEU+choix_color, 700, 40, GLED_ON, 1);
        positionControl(ID_HAUT + choix_color, 214, 40, GLED_ON, 1);
        SendCharCan(0x35,0x02);


    }



}

int algo_carre (int nombre)
{
    switch (nombre) {
        case 0:
            if(resultat==Mycolor) {

                tab[2]=rouge;
            } else {
                tab[2]=Mycolor;
            }
            break;

        case 2:

            if(resultat==Mycolor) {

                tab[0]=rouge;
            } else {
                tab[1]=Mycolor;
            }
            break;
        case 3:

            if(resultat==Mycolor) {
                tab[4]=Enemy_color;
                tab[5]=Enemy_color;
                tab[6]=Mycolor;
            } else {
                tab[4]=Mycolor;
                tab[5]=Mycolor;
                tab[6]=Enemy_color;
            }
            break;
        case 4:

            if(resultat==Mycolor) {
                tab[3]=Enemy_color;
                tab[5]=Mycolor;
                tab[6]=Enemy_color;
            } else {
                tab[3]=Mycolor;
                tab[5]=Enemy_color;
                tab[6]=Mycolor;
            }
            break;
        case 5:

            if(resultat==Mycolor) {
                tab[3]=Enemy_color;
                tab[4]=Mycolor;
                tab[6]=Enemy_color;
            } else {

                tab[3]=Mycolor;
                tab[4]=Enemy_color;
                tab[6]=Mycolor;
            }

            break;
        case 6:

            if(resultat==Mycolor) {

                tab[3]=Mycolor;
                tab[4]=Enemy_color;
                tab[5]=Enemy_color;
            } else {

                tab[3]=Enemy_color;
                tab[4]=Mycolor;
                tab[5]=Mycolor;
            }
            break;
    }

    return 0;
}