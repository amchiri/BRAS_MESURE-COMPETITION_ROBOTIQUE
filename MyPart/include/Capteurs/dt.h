#ifndef DT_H
#define DT_H
#include "main.h"

#define V_min 0.561  //Tension (V) minimale prélevée sur la résistance
#define V_max 3.282   //Tension (V) maximale prélevée sur la résistance

#define D_min 50     //Distance (mm) minimale mesurée par le capteur
#define D_max 1500   //Distance (mm) maximale mesurée par le capteur

#define Nb_echantillon 5 //Nombre de valeurs utilisées pour le moyennage 

#define Correction 0 //Correction apportée sur la formule

#define D_ROULEAU_AXE 200

#define Conv (V_max - V_min)/((V_max/3.3)-(V_min/3.3))

#define A (D_max - D_min)/(V_max - V_min)*Conv
#define B (D_max - ((D_max - D_min)/(V_max - V_min))*V_max) + Correction


void f_mesure(void);
double mediane(double* buff_med, int size_med);
void tri(double* tab, int size);
void interrupt(void);

extern double DT1_trait_Ex;
extern double DT2_trait_Ex;
extern double DT3_trait_Ex;
extern double DT4_trait_Ex;

extern unsigned char DT1_interrupt_Ex;
extern unsigned char DT2_interrupt_Ex;
extern unsigned char DT3_interrupt_Ex;
extern unsigned char DT4_interrupt_Ex;
#endif 