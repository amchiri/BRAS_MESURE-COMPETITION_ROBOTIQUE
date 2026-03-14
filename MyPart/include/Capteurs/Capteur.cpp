#include "Capteur.h"

/////////////////////////////////////Lecture Batterie////////////////////////////
AnalogIn Val_batterie(PC_2);
/////////////////////////////////////Capteurs monocouleur////////////////////////
DigitalIn monocouleur1(PC_15);
DigitalIn monocouleur2(PC_14);
DigitalIn monocouleur3(PB_10);
DigitalIn monocouleur4(PC_3);

short distance_moyenne;

short lecture_telemetre(char numero_telemetre)  // DEGUEUX MAIS FONCTIONNEL :')
{
    switch(numero_telemetre) {
        case 1:
            distance_moyenne=(short)DT1_trait_Ex;
            break;

        case 2:
            distance_moyenne=(short)DT2_trait_Ex;
            break;

        case 3:
            distance_moyenne=(short)DT3_trait_Ex;
            break;

        case 4:
            distance_moyenne=(short)DT4_trait_Ex;
            break;
    }
    return distance_moyenne;
}