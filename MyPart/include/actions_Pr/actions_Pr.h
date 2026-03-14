#ifndef ACTIONS_PR_H
#define ACTIONS_PR_H
#include "main.h"

#define rouge 0
#define Enemy_color 2
#define Mycolor 1

extern char aut_bras_av_at, aut_bras_av_re;
extern char aut_bras_av_1_at, aut_bras_av_1_re;
extern char aut_bras_av_2_at, aut_bras_av_2_re;
extern char aut_bras_av_3_at, aut_bras_av_3_re;
extern char aut_manche_haut, aut_manche_bas, aut_manche_moy ;
extern char aut_bras_av_prepa, aut_bras_av_pose;
extern char aut_bras_av_1_prepa, aut_bras_av_1_pose;
extern char aut_bras_av_2_prepa, aut_bras_av_2_pose;
extern char aut_bras_av_3_prepa, aut_bras_av_3_pose;

//////////////////////////à garder//////////////////////////
void gabarit_robot_droit(void);
void gabarit_robot_gauche(void);
void gabarit_robot_manche(void);
void pavilon_deploye(void);

//////////////////////////test//////////////////////////////
void test_BRAS_A(void);
void test_BRAS_B(void);
void test_BRAS_C(void);
void test_BRAS_D(void);

void test_BRAS_1(void);
void test_BRAS_2(void);
void test_BRAS_3(void);
void test_BRAS_4(void);
void test_BRAS_5(void);
void test_BRAS_6(void);

//////////////////////////selection/////////////////////////
void selection_bras_attraper(void);
void selection_bras_relacher(void);

void selection_bras_prepa(void);
void selection_bras_poser(void);

//////////////////////////automates/////////////////////////
void automate_bras_attraper_1(void);
void automate_bras_relacher_1(void);
void automate_bras_attraper_2(void);
void automate_bras_relacher_2(void);
void automate_bras_attraper_3(void);
void automate_bras_relacher_3(void);
void automate_manche_air_haut(void) ;
void automate_manche_air_bas(void) ;
void automate_manche_air_moy(void) ;
void automate_bras_prepa_1(void);
void automate_bras_poser_1(void);
void automate_bras_prepa_2(void);
void automate_bras_poser_2(void);
void automate_bras_prepa_3(void);
void automate_bras_poser_3(void);

int mesure_resistor(float vpot,bool color);        
int traitement ();
void set_color(bool color);
int algo_carre (int nombre);
void BF_test_mesure (bool BJ);
/*void automate_position_lidar(void);*/
#endif