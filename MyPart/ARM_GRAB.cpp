#include "mbed.h"
#include "fonctions_herkulex.h"
#include <cstdio>
#include <string>

using namespace std::chrono_literals;

#define PLAYTIME 40
#define SERIAL 1

// ID des servos, à verifier ou modifier si besoin sur Herkulex Manager
//  baudrate à 115200
#define ID_BASE 100
#define ID_HAUT 102
#define ID_MILLIEU 101

#define rouge 0
#define Enemy_color 2
#define Mycolor 1

AnalogIn vpot(p20);   // le point de tension mesuré
DigitalOut led(LED1);  // led d'indication
DigitalOut led1(LED2); // led d'indication
DigitalIn button(p5);  // boutton qui a
DigitalIn color(p7);   // interrupteur qui indique si on est dans le camp jaune ou violet 1=jaune 0=violet
BufferedSerial pc(USBTX, USBRX, 9600);

void preselection_tableau(int nb, int tab[5]); // definition des fonctions
void utilisation_tab(int tabref[4][6]);        // definition des fonctions
void mesure_resistor(void);                    // definition des fonctions

int tabref[4][6] = {Mycolor, Mycolor, rouge, Enemy_color, Mycolor, 0,  // paterne 0
                    rouge, Mycolor, Mycolor, Mycolor, Enemy_color, 0,  // paterne 1
                    Mycolor, Mycolor, rouge, Mycolor, Enemy_color, 0,  // paterne 2
                    rouge, Mycolor, Mycolor, Enemy_color, Mycolor, 0}; // paterne 3
int tab[5];
bool flag_cpt3 = false,
     besoin = true; // par defaut true par ce que on s'apprete deja a mesurer
int cpt = 0;        // le nombre de cpt represente le nombbre sur quel carré de fouilles on est actuellement
int choix;          // le choix est une variable qui prendra sois 1,2,3 et 4 qui indique quel paterne predefinis on prendra selon les resistance mmesuré
int main(void)
{
    setTorque(ID_BASE, TORQUE_ON, SERIAL);
    setTorque(ID_HAUT, TORQUE_ON, SERIAL);
    setTorque(ID_MILLIEU, TORQUE_ON, SERIAL);
    // on positione le bras pour qu'il mesure
    positionControl(ID_HAUT, 630, PLAYTIME, GLED_ON, SERIAL);    // puis on positionne le hau
    positionControl(ID_MILLIEU, 770, PLAYTIME, GLED_ON, SERIAL); // on positionne la base
    positionControl(ID_BASE, 532, PLAYTIME, GLED_ON, SERIAL);    // puis on positionne le haut

    while (true)
    {
        if ((button == 1) || (cpt == 0)) // ce if sert juste pour bien presenter la macket qu'on fasse étape par étape
        {
            if(button==0){
            led1 = 0; // reset des indications (bleu)
            led = 0;  // reset des indications (rouge)

            if (besoin == true) // si besoin est on mesure la résistance
            {
                ThisThread::sleep_for(50ms);
                mesure_resistor();                  // mesure de la resistance en contact

            }
        if(besoin == false){ 
            switch (tabref[choix][cpt])
            {
            case rouge:
                led = 1;      // indication qu'on ignore le carré de fouille
                if (cpt == 0) // par defaut on connait le debut de paterne d'aprés la premiere resistance mesuré
                {
                    // on remet le bras a sa postition initiale
                    positionControl(ID_BASE, 456, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_HAUT, 691, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_MILLIEU, 743, PLAYTIME, GLED_ON, SERIAL);
                    
                    cpt = cpt + 1; // coup suivant
                }
                else
                {
                    tab[cpt]=rouge;
                    printf("test ok");

                    cpt = cpt + 1; // coup suivant
                }

                break;

            case Mycolor:
                if (cpt == 0) // par defaut on connait le debut de paterne d'aprés la premiere resistance mesuré
                {
                    positionControl(ID_BASE, 429, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_HAUT, 94, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_MILLIEU, 959, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_BASE, 620, PLAYTIME, GLED_ON, SERIAL);
                    // on bascule le carré de fouille
                    positionControl(ID_MILLIEU, 512, PLAYTIME, GLED_ON, SERIAL);
                    // on le remet a sa postition initiale
                    positionControl(ID_BASE, 456, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_HAUT, 691, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_MILLIEU, 743, PLAYTIME, GLED_ON, SERIAL);
                    cpt = cpt + 1; // coup suivant
                }
                else if ((cpt == 3) && (flag_cpt3 == false)) // c'est le moment ou on connait plus le chemin donc il faut mesure
                {
                    // on positione le bras pour qu'il mesure
                    positionControl(ID_MILLIEU, 770, PLAYTIME, GLED_ON, SERIAL); // on positionne la base
                    positionControl(ID_HAUT, 630, PLAYTIME, GLED_ON, SERIAL);    // puis on positionne le hau
                    positionControl(ID_BASE, 532, PLAYTIME, GLED_ON, SERIAL);    // puis on positionne le haut
                    besoin = true;                                               // on le remet a true par ce que on besoin de mesure la resitance
                    flag_cpt3 = true;                                            // ce flag veut juste indiquer qu'on a mesure dans plus besoin de rentrer dans ce if
                }

                else
                {
                    // on positionne le bras
                    positionControl(ID_BASE, 429, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_HAUT, 94, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_MILLIEU, 959, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_BASE, 620, PLAYTIME, GLED_ON, SERIAL);
                      // Puis on bascule le carré de fouille
                    positionControl(ID_MILLIEU, 512, PLAYTIME, GLED_ON, SERIAL);
                    // on le remet a sa postition initiale
                    positionControl(ID_BASE, 456, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_HAUT, 691, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_MILLIEU, 743, PLAYTIME, GLED_ON, SERIAL);
                    tab[cpt]=Mycolor;
                    cpt = cpt + 1; // coup suivant
                    
                }

                break;

            case Enemy_color:
                if ((cpt == 3) && (flag_cpt3 == false)) // c'est le moment ou on connait plus le chemin donc il faut mesurer
                {
                    // on positione le bras pour qu'il mesure
                    positionControl(ID_MILLIEU, 770, PLAYTIME, GLED_ON, SERIAL); // on positionne la base
                    positionControl(ID_HAUT, 630, PLAYTIME, GLED_ON, SERIAL);    // puis on positionne le hau
                    positionControl(ID_BASE, 532, PLAYTIME, GLED_ON, SERIAL);    // puis on positionne le haut
                    besoin = true;
                    flag_cpt3 = true;
                }
                else
                {
                    led1 = 1;                               // indication qu'on ignore le carré de fouille
                    // on le remet a sa postition initiale en cas ou du (cpt == 3)
                    positionControl(ID_BASE, 456, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_HAUT, 691, PLAYTIME, GLED_ON, SERIAL);
                    positionControl(ID_MILLIEU, 743, PLAYTIME, GLED_ON, SERIAL);
                    tab[cpt]=Enemy_color;
                    cpt = cpt + 1; // coup suivant
                }
                break;
            };
            }
            }
        }
    }
}

void preselection_tableau(int nb, int tab[5]) // definition de la fonction élimnation des différent paterne des carrés de fouilles
{
    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < nb; i++)
        {
            if (tabref[j][i] != tab[i]) // on compare notre paterne actuel avec celui des prédéfinis
            {
                tabref[j][5] = 1;
            }
        }
    }
}

void utilisation_tab(int tabref[4][6]) // definition de la fonction selection du paterne le plus juste du paterne qu'on essaye de déterminer
{
    if (cpt < 3) // Car pour cpt<3 il y a deux paternes chacun dans les 4 paternes qui sont les même
    {
        if ((tabref[0][5] == 0) && (tabref[2][5] == 0))
        {
            choix = 0; // ce chhoix nous sera utile jusqu'a que cpt atteint 3
        }
        else if ((tabref[1][5] == 0) && (tabref[3][5] == 0))
        {
            choix = 1; // ce chhoix nous sera utile jusqu'a que cpt atteint 3
        }
        printf("%d",choix);
    }
    else // sinon on voit pour chaque paterne
    {
        for (int i = 0; i < 4; i++)
        {
            if (tabref[i][5] == 0)
            {
                choix = i; // ce choix dis que le paterne qu'on parcour est le paterne i
                return;
            }
        }
    }
}

void mesure_resistor(void) // definition de la fontion qui mesure les resistances
{
    float sample = vpot;
    if ((sample > 0.8) && (sample < 0.87)) // marge d'ereur  +-5% de ce que qu'on mesure de la resistance bleu
    {
        if (color == 1)
        {
            tab[cpt] = Enemy_color;
  
        }
        else
        {
            tab[cpt] = Mycolor;
        
        }
                        preselection_tableau(cpt+1, tab); //élimnation des différent paterne des carrés de fouilles
                utilisation_tab(tabref);            // selection du paterne le plus juste du paterne qu'on essaye de déterminer
        besoin = false; 
    }
    else if ((sample > 0.37) && (sample < 0.42)) // marge d'ereur +-5% de ce que qu'on mesure de la resistance rouge
    {
        tab[cpt] = rouge;
        besoin = false; 
                        preselection_tableau(cpt+1, tab); //élimnation des différent paterne des carrés de fouilles
                utilisation_tab(tabref);            // selection du paterne le plus juste du paterne qu'on essaye de déterminer
    }
    else if ((sample > 0.70) && (sample < 0.78)) // marge d'ereur +-5% de ce que qu'on mesure de la resistance jaune
    {
        if (color == 1)
        {
            tab[cpt] = Mycolor;
        }
        else
        {
            tab[cpt] = Enemy_color;
        }
                        preselection_tableau(cpt+1, tab); //élimnation des différent paterne des carrés de fouilles
                utilisation_tab(tabref);            // selection du paterne le plus juste du paterne qu'on essaye de déterminer
        besoin = false; 
    }
}