#include "mbed.h"
#include "fonctions_herkulex.h"
//#include "ident_crac.h"

Timer Time_out;

//-------------------------Définition des ports série---------------------------
//UnbufferedSerial pc(USBTX, USBRX,115200);

UnbufferedSerial serial1(PB_6,PB_7,115200);               //              P : 41 et 42
UnbufferedSerial serial2(PC_12,PD_2,115200);             //              P : 43 et 47
UnbufferedSerial serial3(PC_10,PC_11,115200);             //              P : 44 et 48
UnbufferedSerial serial4(PC_6,PC_7,115200 );               //  sans le drapreu 57600 et avec c'est comme les autres            P : 45
UnbufferedSerial serial5(PA_0,PA_1,115200);               //              P : 46


//----------------------------variables de reception----------------------------
uint8_t rx[300];
uint8_t rx2[256];
unsigned char size_reponse=100;
unsigned char recevoir = 0;
float tempo;
unsigned char i2 = 0;
unsigned char flag_serial1_receive2 = 0;
//--------------------variables et fonction de verification---------------------
#define tolerance_en_position 16           //1 degre=(1002-21)/320=3.066position
#define tolerance_en_position_negatif -16
#define B_tolerance_en_position 16           //1 degre=(1002-21)/320=3.066position
#define B_tolerance_en_position_negatif -16
#define V_b 45              //temps d'attente de bras
#define V_m 45              //temps d'attente de bras
#define V_h 45              //temps d'attente de bras
#define TEMPO_R 16            //temps d'attente de reception
#define PWM_recl 0.6
#define temps_servo_G 65     //temps d'attente des servos sauf 2 bras
#define new_tempoX 45        //temps d'attente de correction double


int16_t pos_position = 0, get_pos = 0, pos_ID = 0;
uint8_t pos_led = 0, Status = 0,iID = 0;
uint8_t nombre_servo = 0;
uint8_t pos_time = 0;
uint16_t position_servo_mul[20];
uint8_t data_servo_mul[40];
uint8_t flag_correction = 0;
float new_tempo=0;
float tab_tempo[20];
uint16_t position_servo_mul_different[20];
uint8_t data_servo_mul_different[60];
int8_t my_Tor = 0;
int8_t Tension_inter = 0;
double Tension = 0;
uint8_t coeffient_time = 1;
uint8_t veri = 0;
typedef enum {pos,vitesse,pos_mul_complex,pos_mul_complex_different} type_etat ;
unsigned char serial_numero = 0;
static type_etat etat=pos;
int nb_erreur_pas_de_couple = 0;
int nb_erreur_pos = 0;
int cpt_out = 0;



//---------------------fonction d'interruption de reception de serial1---------------------
/*
 *Ici on crée une interruption afin de rendre prioritaire la réception de données
 *!!! Pour utiliser les fonctions utilisant les interruptions, il faut 'activer' ces dernières
 *!!!avec la fonction servo_interrupt_en();
 */
unsigned char flag_perdu_info_serial1 = 0, indicateur_serial1 = 0, Size_trame_serial1 = 0, old_valueserial1 = 0;
unsigned char char_receive_pc[100];
unsigned char char_receive_serial1[100];
unsigned char valueserial1=0;
unsigned char valuepc=0,flag_seconde=0,flag_pc_receive=0,flag_serial1_receive=0;
int pospos;

//
//La fonction receive_serial1() est appelée par la fonction d'interruption
//elle appelle la fonction automate_serial() et range les données reçues dans un tableau
//
void receive_serial1()
{
    char_receive_serial1[valueserial1]=serial1.getc();
    automate_serial1();
}
//
//fonction d'interruption
// elle se déclenche dès que des données se trouvent sur le port de réception
// elle appelle alors immédiatement la fonction receive_serial1();
//
void Interrupt1_en(void)
{
    serial1.attach(&receive_serial1,Serial::RxIrq);
}
bool verification()
{
    uint8_t i = 0;
    switch(etat) {
        case pos:
            //------------------------Status--------------------
            Status = getStatus(pos_ID,serial_numero);
            wait_ms(3);
            ////pc.printf("status = %d",Status);
            switch(Status) {
                case 0:
                    break;

                case 2: //Exceed allowed POT limit
                    //pc.printf("ERR-Depasse la limite de position\n");
                    //clean_ERR(pos_ID);
                    //wait_ms(500);
                    clear(pos_ID,serial_numero);
                    //positionControl(pos_ID, 1000, 3, GLED_ON);
                    wait_ms(3);
                    Status = getStatus(pos_ID,serial_numero);
                    wait_ms(3);
                    ////pc.printf("status = %d",Status);
                    break;
            }
            //------------------Torque et position------------------------------
            my_Tor = Get_Torque(pos_ID,serial_numero);
            wait_ms(5);
            ////pc.printf("my_Tor = %x\n",my_Tor);
            while(my_Tor != 0x60) {
                setTorque(pos_ID,TORQUE_ON,serial_numero);
                my_Tor = Get_Torque(pos_ID,serial_numero);
                wait_ms(5);
            }
            Tension_inter = Get_Tension_actuelle(pos_ID,serial_numero);
            Tension = Tension_inter*0.074;
            if(Tension <=6.60) {
                coeffient_time = 6;
            } else if(Tension <= 6.90) {
                coeffient_time = 4;
            } else if(Tension <= 7.10) {
                coeffient_time = 2;
            } else if(Tension > 7.10) {
                coeffient_time = 1;
            }
            get_pos = getPos(pos_ID,serial_numero);
            //pc.printf("P4=%d   ",get_pos);
            if(((get_pos - pos_position)>tolerance_en_position)||((get_pos - pos_position)<tolerance_en_position_negatif)) {
                if((get_pos - pos_position)>tolerance_en_position) {
                    new_tempo=(get_pos - pos_position)*0.084*coeffient_time + 1;
                    if (new_tempo > 254) new_tempo = 254;
                } else if((get_pos - pos_position)<tolerance_en_position_negatif) {
                    new_tempo=(get_pos - pos_position)*0.084*coeffient_time +1;
                    if (new_tempo > 254) new_tempo = 254;
                }
                positionControl(pos_ID, pos_position, new_tempo, pos_led,serial_numero);
                //pc.printf("Correction!\n");
            }
            break;
        case pos_mul_complex:
            //---------------------------Status---------------------------
            for(i=0; i<nombre_servo; i++) 
            {
                Status = getStatus(data_servo_mul[1+2*i],serial_numero);
                //pc.printf("status = %d",Status);
                switch(Status) {
                    case 0:
                        break;

                    case 2: //Exceed allowed POT limit
                        ////pc.printf("ERR-Depasse la limite de position\n");
                        //clean_ERR(id);
                        //wait_ms(500);
                        clear(data_servo_mul[1+2*i],serial_numero);
                        //positionControl(id, 1000, 3, GLED_ON);
                        //wait_ms(3);
                        //Status = getStatus(data_servo_mul[1+2*i]);
                        //wait_ms(3);
                        ////pc.printf("status = %d",Status);
                        break;
                }
            }
            //----------------------Torque et position--------------------------
            for(i=0; i<nombre_servo; i++) 
            {
                my_Tor = Get_Torque(data_servo_mul[1+2*i],serial_numero);
                //Time_out.reset();
                //Time_out.start();
                while((my_Tor != 0x60))//&&(Time_out.read_ms()<100) )
                {
                    setTorque(data_servo_mul[1+2*i],TORQUE_ON,serial_numero);
                    my_Tor = Get_Torque(data_servo_mul[1+2*i],serial_numero);
                    ////pc.printf(" SET_TORQUE   ");

                    Status = getStatus(data_servo_mul[1+2*i],serial_numero);
                    clear(data_servo_mul[1+2*i],serial_numero);
                    Status = getStatus(data_servo_mul[1+2*i],serial_numero);
                }
                //Time_out.stop();
            }
            veri = 0;
            //Time_out.reset();
            //Time_out.start();
            cpt_out = 0;
                while((veri < nombre_servo) && cpt_out < 5)//&&(Time_out.read_ms()<100))
                {
                    cpt_out++;
                    for(i=0; i<nombre_servo; i++) 
                    {
                        my_Tor = Get_Torque(data_servo_mul[1+2*i],serial_numero);
                        //Time_out.reset();
                        //Time_out.start();
                        while((my_Tor != 0x60))//&&(Time_out.read_ms()<100))
                        {
                            setTorque(data_servo_mul[1+2*i],TORQUE_ON,serial_numero);
                            my_Tor = Get_Torque(data_servo_mul[1+2*i],serial_numero);
                            ////pc.printf(" SET_TORQUE   ");
    
                            Status = getStatus(data_servo_mul[1+2*i],serial_numero);
                            clear(data_servo_mul[1+2*i],serial_numero);
                            Status = getStatus(data_servo_mul[1+2*i],serial_numero);
                        }
                        //Time_out.stop();
                    }
                    for(i=0; i<nombre_servo; i++) 
                    {
                        Tension_inter = Get_Tension_actuelle(data_servo_mul[1+2*i],serial_numero);
                        Tension = Tension_inter*0.074;
                        if(Tension <=6.60) {
                            coeffient_time = 6;
                        } else if(Tension <= 6.90) {
                            coeffient_time = 4;
                        } else if(Tension <= 7.10) {
                            coeffient_time = 2;
                        } else if(Tension > 7.10) {
                            coeffient_time = 1;
                        }
                        get_pos = getPos(data_servo_mul[1+2*i],serial_numero);
                        //pc.printf("PosiM=%d   ",get_pos);
                        if((get_pos - position_servo_mul[i])>tolerance_en_position) 
                        {
                            tab_tempo[i]=(get_pos - position_servo_mul[i])*0.084*coeffient_time+1;     //MinTempo pour 1 position:((320/60)*0.17)/(1000-20)=0.94ms  MinPlayTime pour 1 position:0.94ms/11.2ms=0.084
                            if (tab_tempo[i] > 254) tab_tempo[i] = 254;
                            flag_correction = 1;
                        } 
                        else if((get_pos - position_servo_mul[i])<tolerance_en_position_negatif) 
                        {
                            tab_tempo[i]=(position_servo_mul[i] - get_pos)*0.084*coeffient_time+1;
                            if (tab_tempo[i] > 254) tab_tempo[i] = 254;
                            flag_correction = 1;
                        }
                    }
                    if(flag_correction == 1) 
                    {
                        new_tempo = 0;
                        for(i=0; i<nombre_servo; i++) 
                        {
                            if(tab_tempo[i]>new_tempo) 
                            {
                                new_tempo = tab_tempo[i];
                            }
                        }
                        flag_correction = 0;
                        positionControl_Mul_ensemble_complex(nombre_servo,new_tempo,data_servo_mul, position_servo_mul,serial_numero);
                        //pc.printf("Correction!\n");
                    }
                    veri=0;
                    for(i=0; i<nombre_servo; i++)
                    {
                        get_pos = getPos(data_servo_mul[1+2*i],serial_numero);
                        //pc.printf("PosiM=%d   ",get_pos);
                        if((get_pos - position_servo_mul[i])>tolerance_en_position) 
                        {
                            tab_tempo[i]=(get_pos - position_servo_mul[i])*0.084*coeffient_time+1;     //MinTempo pour 1 position:((320/60)*0.17)/(1000-20)=0.94ms  MinPlayTime pour 1 position:0.94ms/11.2ms=0.084
                            if (tab_tempo[i] > 254) tab_tempo[i] = 254;
                            flag_correction = 1;
                        } 
                        else if((get_pos - position_servo_mul[i])<tolerance_en_position_negatif) 
                        {
                            tab_tempo[i]=(position_servo_mul[i] - get_pos)*0.084*coeffient_time+1;
                            if (tab_tempo[i] > 254) tab_tempo[i] = 254;
                            flag_correction = 1;
                        }
                        else { //if(((get_pos - position_servo_mul[i])<tolerance_en_position)&&((get_pos - position_servo_mul[i])>tolerance_en_position_negatif))
                            veri++;
                        }
                    }
                }
                //Time_out.stop();
            break;           
        case pos_mul_complex_different:
            //---------------------------Status---------------------------
            for(i=0; i<nombre_servo; i++) {
                Status = getStatus(data_servo_mul_different[1+3*i],serial_numero);
                ////pc.printf("status = %d",Status);
                switch(Status) {
                    case 0:
                        break;

                    case 2: //Exceed allowed POT limit
                        ////pc.printf("ERR-Depasse la limite de position\n");
                        //clean_ERR(id);
                        //wait_ms(500);
                        clear(data_servo_mul_different[1+3*i],serial_numero);
                        //positionControl(id, 1000, 3, GLED_ON);
                        //wait_ms(3);
                        //Status = getStatus(data_servo_mul_different[1+2*i]);
                        //wait_ms(3);
                        ////pc.printf("status = %d",Status);
                        break;
                }
            }
            //-------------------Torque et position-----------------------------
            for(i=0; i<nombre_servo; i++) {
                my_Tor = Get_Torque(data_servo_mul_different[1+3*i],serial_numero);
                while(my_Tor != 0x60) {
                    setTorque(data_servo_mul_different[1+3*i],TORQUE_ON,serial_numero);
                    my_Tor = Get_Torque(data_servo_mul_different[1+3*i],serial_numero);
                    //wait_ms(5);
                    ////pc.printf(" SET_TORQUE   ");
                }
            }
            for(i=0; i<nombre_servo; i++) {
                Tension_inter = Get_Tension_actuelle(data_servo_mul_different[1+3*i],serial_numero);
                Tension = Tension_inter*0.074;
                if(Tension <=6.60) {
                    coeffient_time = 6;
                } else if(Tension <= 6.90) {
                    coeffient_time = 4;
                } else if(Tension <= 7.10) {
                    coeffient_time = 2;
                } else if(Tension > 7.10) {
                    coeffient_time = 1;
                }
                get_pos = getPos(data_servo_mul_different[1+3*i],serial_numero);
                //pc.printf("PosiM=%d   ",get_pos);
                if((get_pos - position_servo_mul_different[i])>tolerance_en_position) {
                    tab_tempo[i]=(get_pos - position_servo_mul_different[i])*0.084*coeffient_time+1;     //MinTempo pour 1 position:((320/60)*0.17)/(1000-20)=0.94ms  MinPlayTime pour 1 position:0.94ms/11.2ms=0.084
                    if (tab_tempo[i] > 254) tab_tempo[i] = 254;
                    data_servo_mul_different[2+3*i] = tab_tempo[i];
                    flag_correction = 1;
                } else if((get_pos - position_servo_mul_different[i])<tolerance_en_position_negatif) {
                    tab_tempo[i]=(position_servo_mul_different[i] - get_pos)*0.084*coeffient_time+1;
                    if (tab_tempo[i] > 254) tab_tempo[i] = 254;
                    data_servo_mul_different[2+3*i] = tab_tempo[i];
                    flag_correction = 1;
                }
            }
            if(flag_correction == 1) {
                flag_correction = 0;
                positionControl_Mul_ensemble_different_complex(nombre_servo,data_servo_mul_different, position_servo_mul_different,serial_numero);
                //pc.printf("Correction!\n");
            }
            break;
    }
   return true;
}
//
//La fonction verification_3_bras() sert à vérifier les bras utilisés après un mouvement groupé
//Elle change les numéros de sérial et des cerveaux moteurs
//
void verification_3_bras(uint8_t sens_avant)
{
        uint8_t i=0, idata=0;
        uint8_t servos_bras_avant[6] = {RLED_ON, 1, GLED_ON, 2, BLED_ON, 3}; 
        uint8_t servos_bras_arriere[6] = {RLED_ON, 4, GLED_ON, 5, BLED_ON, 6}; 
        if(sens_avant==1)
        {  
            for(idata=0;idata<6;idata++)
            {
                data_servo_mul[idata] = servos_bras_avant[idata];
            }
        }
        else if(sens_avant==0)
        {
            for(idata=0;idata<6;idata++)
            {
                data_servo_mul[idata] = servos_bras_arriere[idata];
            }         
        }
        
        for(i=0; i<3; i++)
        {
            serial_numero=i+1;
            verification();
        }    
}

//
//La fonction automate_serial1() sert à vérifier la bonne réception des données
//elle est automatiquement appelée par la fonction receive_serial1()
//
void automate_serial1()
{
    typedef enum {Attente,FF,Size,Data} type_etat1;
    static type_etat1 etat1=Attente;
    ///////pc.printf("coucou");

//////pc.printf("%d\r\n", char_receive_serial1[valueserial1]);

    switch (etat1) {
        // état Attente
        //on attend la réception des données
        //si on reçois l'octet 0xFF, il s'agit d'un début de trame
        //on passe à l'état suivant
        //
        case Attente:
            if(char_receive_serial1[0] == 0xFF) {
                etat1 = FF;
                valueserial1 = 1;
            }
            break;
        // état FF
        //on attend le second octet 0xFF pour confirmer qu'il s'agit d'une trame
        //si on reçoit l'octet 0xFF, il s'agit bien d'une trame Herkulex
        //on passe à l'état suivant
        //Sinon on retourne à l'état précédent
        //
        case FF:
            if(char_receive_serial1[1] == 0xFF) {
                etat1 = Size;
                valueserial1 = 2;
            } else {
                etat1 = Attente;
                valueserial1 = 0;
                flag_perdu_info_serial1 = 1;   //flag_perdu_info_serial1
            }
            break;
        // état size
        //On vérifie si l'octet size est supérieur à la taille minimale d'une trame Herkulex,
        //Si oui on passe à l'état suivant
        //Si non on passe à l'état attente et flag_perdu_info_serial1 passe à 1 pour signaler la perte d'information
        case Size:
            if(char_receive_serial1[2] < 7) {
                etat1 = Attente;
                valueserial1 = 0;
                flag_perdu_info_serial1 = 1;   //flag_perdu_info_serial1
            } else {
                etat1 = Data;
                old_valueserial1 = 2;
                valueserial1 = 3;
            }
            Size_trame_serial1 = char_receive_serial1[2];
            break;
        //état data
        //on verifie que la taille de la trame reçue correspond à celle indiquée dans l'octet 'size'
        //si oui
        //flag_serial1_receive passe à 1 pour indiquer que la trame à bien été transmise
        case Data:
            if((valueserial1-2)==(Size_trame_serial1-3)) {
                flag_serial1_receive = 1;
                etat1 = Attente;
                valueserial1 = 0;
            } else {
                valueserial1++;
            }
            break;

        default:
            break;
    }
}
//---------------------fonction d'interruption de reception de serial2---------------------
//même principe que la fonction d'interrutpion de serial1
unsigned char flag_perdu_info_serial2 = 0, indicateur_serial2 = 0, Size_trame_serial2 = 0, old_valueserial2 = 0;
unsigned char char_receive_serial2[100];
unsigned char valueserial2=0;
unsigned char flag_serial2_receive=0;
void receive_serial2()
{
    char_receive_serial2[valueserial2]=serial2.getc();
    automate_serial2();
}

void Interrupt2_en(void)
{
    serial2.attach(&receive_serial2,Serial::RxIrq);
}

void automate_serial2()
{
    typedef enum {Attente,FF,Size,Data} type_etat2;
    static type_etat2 etat2=Attente;
    //////////pc.printf("coucou");

//////pc.printf("%d\r\n", char_receive_serial2[valueserial2]);

    switch (etat2) {
        case Attente:
            if(char_receive_serial2[0] == 0xFF) {
                etat2 = FF;
                valueserial2 = 1;
            }
            break;
        case FF:
            if(char_receive_serial2[1] == 0xFF) {
                etat2 = Size;
                valueserial2 = 2;
            } else {
                etat2 = Attente;
                valueserial2 = 0;
                flag_perdu_info_serial2 = 1;   //flag_perdu_info_serial1
            }
            break;
        case Size:
            if(char_receive_serial2[2] < 7) {
                etat2 = Attente;
                valueserial2 = 0;
                flag_perdu_info_serial2 = 1;   //flag_perdu_info_serial1
            } else {
                etat2 = Data;
                old_valueserial2 = 2;
                valueserial2 = 3;
            }
            Size_trame_serial2 = char_receive_serial2[2];
            break;

        case Data:
            if((valueserial2-2)==(Size_trame_serial2-3)) {
                flag_serial2_receive = 1;
                etat2 = Attente;
                valueserial2 = 0;
            } else {
                valueserial2++;
            }
            break;

        default:
            break;
    }
}
//---------------------fonction d'interruption de reception de serial3---------------------
//même principe que la fonction d'interrutpion de serial1
unsigned char flag_perdu_info_serial3 = 0, indicateur_serial3 = 0, Size_trame_serial3 = 0, old_valueserial3 = 0;
unsigned char char_receive_serial3[100];
unsigned char valueserial3=0;
unsigned char flag_serial3_receive=0;
void receive_serial3()
{
    char_receive_serial3[valueserial3]=serial3.getc();
    automate_serial3();
}

void Interrupt3_en(void)
{
    serial3.attach(&receive_serial3,Serial::RxIrq);
}

void automate_serial3()
{
    typedef enum {Attente,FF,Size,Data} type_etat3;
    static type_etat3 etat3=Attente;
    //////////pc.printf("coucou");

//////pc.printf("%d\r\n", char_receive_serial3[valueserial3]);

    switch (etat3) {
        case Attente:
            if(char_receive_serial3[0] == 0xFF) {
                etat3 = FF;
                valueserial3 = 1;
            }
            break;
        case FF:
            if(char_receive_serial3[1] == 0xFF) {
                etat3 = Size;
                valueserial3 = 2;
            } else {
                etat3 = Attente;
                valueserial3 = 0;
                flag_perdu_info_serial3 = 1;   //flag_perdu_info_serial1
            }
            break;
        case Size:
            if(char_receive_serial3[2] < 7) {
                etat3 = Attente;
                valueserial3 = 0;
                flag_perdu_info_serial3 = 1;   //flag_perdu_info_serial1
            } else {
                etat3 = Data;
                old_valueserial3 = 2;
                valueserial3 = 3;
            }
            Size_trame_serial3 = char_receive_serial3[2];
            break;

        case Data:
            if((valueserial3-2)==(Size_trame_serial3-3)) {
                flag_serial3_receive = 1;
                etat3 = Attente;
                valueserial3 = 0;
            } else {
                valueserial3++;
            }
            break;

        default:
            break;
    }
}
//---------------------fonction d'interruption de reception de serial4---------------------
//même principe que la fonction d'interrutpion de serial1
unsigned char flag_perdu_info_serial4 = 0, indicateur_serial4 = 0, Size_trame_serial4 = 0, old_valueserial4 = 0;
unsigned char char_receive_serial4[100];
unsigned char valueserial4=0;
unsigned char flag_serial4_receive=0;
void receive_serial4()
{
    char_receive_serial4[valueserial4]=serial4.getc();
    automate_serial4();
}

void Interrupt4_en(void)
{
    serial4.attach(&receive_serial4,Serial::RxIrq);
}

void automate_serial4()
{
    typedef enum {Attente,FF,Size,Data} type_etat4;
    static type_etat4 etat4=Attente;
    //////////pc.printf("coucou");

//////pc.printf("%d\r\n", char_receive_serial4[valueserial4]);

    switch (etat4) {
        case Attente:
            if(char_receive_serial4[0] == 0xFF) {
                etat4 = FF;
                valueserial4 = 1;
            }
            break;
        case FF:
            if(char_receive_serial4[1] == 0xFF) {
                etat4 = Size;
                valueserial4 = 2;
            } else {
                etat4 = Attente;
                valueserial4 = 0;
                flag_perdu_info_serial4 = 1;   //flag_perdu_info_serial1
            }
            break;
        case Size:
            if(char_receive_serial4[2] < 7) {
                etat4 = Attente;
                valueserial4 = 0;
                flag_perdu_info_serial4 = 1;   //flag_perdu_info_serial1
            } else {
                etat4 = Data;
                old_valueserial4 = 2;
                valueserial4 = 3;
            }
            Size_trame_serial4 = char_receive_serial4[2];
            break;

        case Data:
            if((valueserial4-2)==(Size_trame_serial4-3)) {
                flag_serial4_receive = 1;
                etat4 = Attente;
                valueserial4 = 0;
            } else {
                valueserial4++;
            }
            break;

        default:
            break;
    }
}
//---------------------fonction d'interruption de reception de serial5---------------------
//même principe que la fonction d'interrutpion de serial1
unsigned char flag_perdu_info_serial5 = 0, indicateur_serial5 = 0, Size_trame_serial5 = 0, old_valueserial5 = 0;
unsigned char char_receive_serial5[100];
unsigned char valueserial5=0;
unsigned char flag_serial5_receive=0;
void receive_serial5()
{
    char_receive_serial5[valueserial5]=serial5.getc();
    automate_serial5();
}

void Interrupt5_en(void)
{
    serial5.attach(&receive_serial5,Serial::RxIrq);
}

void automate_serial5()
{
    typedef enum {Attente,FF,Size,Data} type_etat5;
    static type_etat5 etat5=Attente;
    //////////pc.printf("coucou");

//////pc.printf("%d\r\n", char_receive_serial5[valueserial5]);

    switch (etat5) {
        case Attente:
            if(char_receive_serial5[0] == 0xFF) {
                etat5 = FF;
                valueserial5 = 1;
            }
            break;
        case FF:
            if(char_receive_serial5[1] == 0xFF) {
                etat5 = Size;
                valueserial5 = 2;
            } else {
                etat5 = Attente;
                valueserial5 = 0;
                flag_perdu_info_serial5 = 1;   //flag_perdu_info_serial1
            }
            break;
        case Size:
            if(char_receive_serial5[2] < 7) {
                etat5 = Attente;
                valueserial5 = 0;
                flag_perdu_info_serial5 = 1;   //flag_perdu_info_serial1
            } else {
                etat5 = Data;
                old_valueserial5 = 2;
                valueserial5 = 3;
            }
            Size_trame_serial5 = char_receive_serial5[2];
            break;

        case Data:
            if((valueserial5-2)==(Size_trame_serial5-3)) {
                flag_serial5_receive = 1;
                etat5 = Attente;
                valueserial5 = 0;
            } else {
                valueserial5++;
            }
            break;

        default:
            break;
    }
}
//----------------xxxxx----fonction de fermture de serial-----------------------
/*void N_Herkulex()
{

if(Sv != NULL)
delete Sv;
if(recevoir==2) {
size_reponse = rx2[recevoir];
}
}*/
//-------------------------fonction de transmission-----------------------------
//
//Permet de transmettre une trame manuellement sur une liaison série choisie
//
//packetSize    ==> Taille totale de la trame en octets
//               en-têtes (HEADER) et données (data) inclus
//
//data          ==> Données ( Ici il d'agit de la trame en entier) à rentrer sous forme de tableau (1 octet par case!)
//
//numero_serial ==> Numéro de la liaison série sur laquelle on
//                  envoie la trame
void txPacket(uint8_t packetSize, uint8_t* data, uint8_t numero_serial)


/*#ifdef HERKULEX_DEBUG
pc->printf("[TX]");
for(uint8_t i = 0; i < packetSize ; i++)
{
#ifdef HERKULEX_DEBUG
pc->printf("%02X ",data[i]);
#endif
txd->putc(data[i]);
}
#ifdef HERKULEX_DEBUG
pc->printf("\n");
#endif*/
{
    serial_numero = numero_serial;
    if(numero_serial == 1) {                              //Envoi sur la liaison série 1
        for(uint8_t i = 0; i < packetSize ; i++) {        //
            while(serial1.writeable() == 0);              //On envoie 1 octet toute les 100 us
            serial1.putc(data[i]);                        //
            wait_us(100);                                 //
        }
    } else if(numero_serial == 2) {                              //Envoi sur la liaison série 2
        for(uint8_t i = 0; i < packetSize ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(data[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {                              //Envoi sur la liaison série 3
        for(uint8_t i = 0; i < packetSize ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(data[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {                              //Envoi sur la liaison série 4
        for(uint8_t i = 0; i < packetSize ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(data[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {                              //Envoi sur la liaison série 5
        for(uint8_t i = 0; i < packetSize ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(data[i]);
            wait_us(100);
        }
    }
    wait_ms(TEMPO_R);
}
//----------------------------fonction de reception-----------------------------
//Permet de recevoir une trame
void rxPacket(uint8_t packetSize, uint8_t* data, uint8_t numero_serial)
//
//packetSize ==> taille de la trame à recevoir
//data       ==> Données
//
{

    /*#ifdef HERKULEX_DEBUG
    pc->printf("[RX]");
    #endif
    for (uint8_t i=0; i < packetSize; i++)
    {
    data[i] = rxd->getc();
    #ifdef HERKULEX_DEBUG
    pc->printf("%02X ",data[i]);
    #endif
    }
    #ifdef HERKULEX_DEBUG
    pc->printf("\n");
    #endif*/
    serial_numero = numero_serial;
    if(numero_serial == 1) {
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                data[i4] = char_receive_serial1[i4];
                //////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                data[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                data[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                data[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                data[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }
}
//----------------------fonction pour sortir de l'état d'erreur-------------------------
//
//Permet de "sortir" de la mise en erreur d'un servomoteur
//
void clear(uint8_t id, uint8_t numero_serial)
//
// id            ==> On entre l'ID du servomoteur que l'on souhaite sortir de l'état d'erreur
// numero serial ==> On entre le numéro de la liaison série sur laquelle se trouve le servomoteur concerné
{
    uint8_t txBuf[11];
    serial_numero = numero_serial;
    txBuf[0] = HEADER;                                // Packet Header (0xFF)
    txBuf[1] = HEADER;                                // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 4;                   // Packet Size
    txBuf[3] = id;                                    // Servo ID
    txBuf[4] = CMD_RAM_WRITE;                         // Command Ram Write (0x03)   *On choisi le CMD pour écrire dans un registre
    txBuf[5] = 0;                                     // Checksum1
    txBuf[6] = 0;                                     // Checksum2
    txBuf[7] = RAM_STATUS_ERROR;                      // Address           *On écrit dans le registre RAM_STATUS_ERROR
    txBuf[8] = BYTE2;                                 // Length                       *
    txBuf[9] = 0;                                     // Clear RAM_STATUS_ERROR
    txBuf[10]= 0;                                     // Clear RAM_STATUS_DETAIL
    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]) & 0xFE; //calcul de checksum1
    txBuf[6] = (~txBuf[5])&0xFE;                                                         //calcul de checksum2
    // send packet (mbed -> herkulex)
    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 11 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 11 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 11 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 11 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 11 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }
    wait_ms(TEMPO_R);
}
//----------------fonction de mise à jour du couple du servo---------------------
void setTorque(uint8_t id, uint8_t cmdTorque, uint8_t numero_serial)
// Permet de modifier l'état du couple d'un servo------------
// id           ==> ID du servomoteur
//cmdTorque     ==> état souhaité pour le couple
//numero_serial ==> Numéro de la liaison série sur laquelle se trouve le servo

// valeurs posssibles pour cmdTorque
// 0x40 Break On        Opérations commandes impossibles
// 0x60 Torque On       Fontionnement normal
// 0x00 Torque Free     Opérations commandes impossibles + possibilité de déplacer le servo manuellement
{
    uint8_t txBuf[10];
    serial_numero = numero_serial;
    txBuf[0] = HEADER;                                       // Packet Header (0xFF)
    txBuf[1] = HEADER;                                       // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 3;                          // Packet Size
    txBuf[3] = id;                                           // Servo ID
    txBuf[4] = CMD_RAM_WRITE;                                // Command Ram Write (0x03)
    txBuf[5] = 0;                                            // Checksum1
    txBuf[6] = 0;                                            // Checksum2
    txBuf[7] = RAM_TORQUE_CONTROL;                           // Address
    txBuf[8] = BYTE1;                                        // Length
    txBuf[9] = cmdTorque;                                    // Torque ON
// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;
// send packet (mbed -> herkulex)
    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }
    wait_ms(TEMPO_R);
}
//-------------fonction de contrôle de position pour un seul servo--------------
//Permet de controler un servomoteur en position
void positionControl(uint8_t id, uint16_t position, uint8_t playtime, uint8_t setLED, uint8_t numero_serial)
//
//id ==> id du servo à déplacer
//position ==> position à atteindre
//playtime ==> temps à mettre pour effectuer le déplacement
//setLED   ==> LED à allumer
//numero-serial ==> numéro de la liaison série
{
    float tempo=0;
    serial_numero = numero_serial;
    //if (position > 1023) return; //1002-21
    if (playtime > 254) playtime = 254; //return; //1-254 == 11.2ms-2.844sec.
    tempo=playtime*0.012;
    pos_ID = id;
    uint8_t txBuf[12];
    etat = pos;
    pos_position = position;
    pos_time = playtime;
    pos_led = setLED;
    txBuf[0] = HEADER;                                    // Packet Header (0xFF)
    txBuf[1] = HEADER;                                    // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 5;                       // Packet Size
    //txBuf[3] = MAX_PID;
    txBuf[3] = id;                                        // pID is total number of servos in the network (0 ~ 253)
    txBuf[4] = CMD_S_JOG;                                 // Command S JOG (0x06)
    txBuf[5] = 0;                                         // Checksum1
    txBuf[6] = 0;                                         // Checksum2
    txBuf[7] = playtime;                                  // Playtime
    txBuf[8] = position & 0x00FF;                         // Position (LSB, Least Significant Bit)
    txBuf[9] =(position & 0xFF00) >> 8;                   // position (MSB, Most Significanct Bit)
    txBuf[10] = POS_MODE | setLED;                        // Pos Mode and LED on/off
    txBuf[11] = id;                                       // Servo ID
// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;
// send packet (mbed -> herkulex)
//txPacket(12, txBuf);
    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 12 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            //pc.printf("%d/",txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 12 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);

            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 12 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 12 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 12 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }
    wait(tempo);
    wait_ms(TEMPO_R);
}
//-------------fonction de controle de vitesse pour un seul servo---------------

void velocityControl(uint8_t id, int16_t speed, uint8_t setLED, uint8_t numero_serial)
//
//id ==> id du servo à déplacer
//speed ==> vitesse (sans dec)
//setLED   ==> LED à allumer
//numero_serial ==> numéro de la liaison série
//
{
    serial_numero = numero_serial;
    if (speed > 1023 || speed < -1023) return;
    uint8_t txBuf[12];
    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 5; // Packet Size
    txBuf[3] = id; // pID is total number of servos in the network (0 ~ 253)
    txBuf[4] = CMD_S_JOG; // Command S JOG (0x06)
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = 0; // Playtime, unmeaningful in turn mode
    if (speed >= 0) {               //On gère la vitesse positive
        txBuf[8] = speed & 0x00FF; // Speed (LSB, Least Significant Bit)
        txBuf[9] =(speed & 0xFF00) >> 8; // Speed (MSB, Most Significanct Bit)
    } else if(speed < 0) {                 //On gère la vitesse négative (voir pg.48 de la documentation herkulex)
        speed= abs(speed);
        txBuf[8] = speed & 0x00FF;         // Speed (LSB, Least Significant Bit)
        txBuf[9] =((speed|0x4000) & 0xFF00) >> 8;  // Speed (MSB, Most Significanct Bit)
    }

    txBuf[10] = TURN_MODE | setLED; // Turn Mode and LED on/off
    txBuf[11] = id; // Servo ID
// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;
// send packet (mbed -> herkulex)
    if(numero_serial == 1) {
        txPacket(12, txBuf,1);
    } else if(numero_serial == 2) {
        txPacket(12, txBuf,2);
    } else if(numero_serial == 3) {
        txPacket(12, txBuf,3);
    } else if(numero_serial == 4) {
        txPacket(12, txBuf,4);
    } else if(numero_serial == 5) {
        txPacket(12, txBuf,5);
    }
    wait_ms(TEMPO_R);
}
//--------------------Compteur de tour------------------------------------------------------------------------
//c'est un compteur de tour (CQFD)
void compteTour(int ID,int16_t speed,uint8_t tour, uint16_t position,uint8_t setLED,uint8_t serial)
//
//id ==> id du servo à déplacer
//speed ==> vitesse
//tour ==> nombre de tour à effectuer
//position ==> position finale
//setLED   ==> LED à allumer
//numero_serial ==> numéro de la liaison série
//
//

{
    int etat =0;
    int flagTour=0;
    int flagDernierTour = 0;
    int end = 0;
    int posAct, posCible, pos;
    servo_interrupt_en();
    clear(ID,serial);
    setTorque(ID, TORQUE_ON,serial);

    posCible=getPos(ID,serial);//position;
    
    //pc.printf("\nPos depart :%d",posCible);
    velocityControl(ID,speed,setLED,serial);
    wait_ms(100);
    if(tour == 0){
        etat = 2;
        tour = 1;
    }
        

    if(speed > 0) {
        while(end != 1) {
            switch (etat) {
                case 0 :
                    pos = getPos(ID,serial);
                    posAct = pos - posCible;
                    if(posAct<0){
                        //pc.printf("\n0 pos :%d",pos);
                        clear(ID,serial);
                        setTorque(ID, TORQUE_ON,serial);
                        etat=1;
                    }
                   
                    break;
                case 1 :
                    pos = getPos(ID,serial);
                    velocityControl(ID,speed,RLED_ON,serial);
                    posAct = posCible - pos;
                    if (posAct < 0 ){
                        //pc.printf("\n1 pos :%d",pos);
                         etat = 2;
                    }
                    break;

                case 2 :

                    clear(ID,serial);
                    setTorque(ID, TORQUE_ON,serial);
                    
                    if (flagTour == tour-1) {
                        if (getPos(ID,serial) > position && !flagDernierTour){
                           //pc.printf("\nTour sup");
                           // flagTour++;
                            flagDernierTour =1;
                            etat =0;
                            posCible = position;
                        }else{
                            //pc.printf("\ngoto pos finale");
                        velocityControl(ID,0,setLED,serial);
                        positionControl(ID,position,1,setLED,serial);
                        end = 1;
                        }
                    }
                    
                    else {
                        flagTour=flagTour+1;
                        //pc.printf("\ntours execute : %d",flagTour);
                        etat = 0;
                    }
                    break;
            }
        }
    } else if(speed < 0) {
        while(end != 1) {
            switch (etat) {
                case 0 :
                    pos = getPos(ID,serial);
                    posAct = pos - posCible;
                    if(posAct>0){
                        //pc.printf("\n0 pos :%d",pos);
                        clear(ID,serial);
                        setTorque(ID, TORQUE_ON,serial);
                        etat=1;
                    }
                   
                    break;

                case 1 :
                    pos = getPos(ID,serial);
                    velocityControl(ID,speed,RLED_ON,serial);
                    posAct = posCible - pos;
                    if (posAct > 0 ){
                        //pc.printf("\n1 pos :%d",pos);
                         etat = 2;
                    }
                    break;

                case 2 :

                    clear(ID,serial);
                    setTorque(ID, TORQUE_ON,serial);
                    
                    if (flagTour == tour-1) {
                        if (getPos(ID,serial) < position && !flagDernierTour){
                           //pc.printf("\nTour sup");
                           // flagTour++;
                            flagDernierTour =1;
                            etat =0;
                            posCible = position;
                        }else{
                            //pc.printf("\ngoto pos finale");
                        velocityControl(ID,0,setLED,serial);
                        positionControl(ID,position,1,setLED,serial);
                        end = 1;
                        }
                    }
                    
                    else {
                        flagTour=flagTour+1;
                        //pc.printf("\ntours execute : %d",flagTour);
                        etat = 0;
                    }
                    break;
            }
        }
    }
}
/*

{
    int etat =0;
    int flagTour=0;
    int end = 0;
    int posAct, posCible;
    servo_interrupt_en();
    clear(ID,serial);
    setTorque(ID, TORQUE_ON,serial);

    posCible=position;
    velocityControl(ID,speed,setLED,serial);
    wait_ms(100);

    if(speed > 0) {
        while(end != 1) {
            switch (etat) {
                case 0 :

                    posAct = getPos(ID,serial);
                    posAct = posAct-posCible;
                    //pc.printf("%d",posAct);
                    if (posAct < 0) {
                        clear(ID,serial);
                        setTorque(ID, TORQUE_ON,serial);
                        etat=1;
                    }
                    break;

                case 1 :

                    velocityControl(ID,speed,RLED_ON,serial);
                    posAct = getPos(ID,serial);
                    posAct = posCible-posAct;
                    if (posAct < 0) etat = 2;
                    break;

                case 2 :

                    clear(ID,serial);
                    setTorque(ID, TORQUE_ON,serial);
                    if (flagTour == tour-1 | tour == 0) {
                        velocityControl(ID,0,setLED,serial);
                        positionControl(ID,posCible,1,setLED,serial);
                        end = 1;

                    } else {
                        flagTour=flagTour+1;
                        etat = 0;
                    }
                    break;
            }
        }
    } else if(speed < 0) {
        while(end != 1) {
            switch (etat) {
                case 0 :

                    posAct = getPos(ID,serial);
                    posAct = posCible-posAct;
                    //pc.printf("%d",posAct);
                    if (posAct < 0) {
                        clear(ID,serial);
                        setTorque(ID, TORQUE_ON,serial);
                        etat=1;
                    }
                    break;

                case 1 :

                    velocityControl(ID,speed,RLED_ON,serial);
                    posAct = getPos(ID,serial);
                    posAct = posAct-posCible;
                    if (posAct < 0) etat = 2;
                    break;

                case 2 :

                    clear(ID,serial);
                    setTorque(ID, TORQUE_ON,serial);
                    if (flagTour == tour-1 | tour == 0) {
                        velocityControl(ID,0,setLED,serial);
                        positionControl(ID,posCible,1,setLED,serial);
                        end =1;
                    } else {
                        flagTour=flagTour+1;
                        etat = 0;
                    }
                    break;
            }
        }
    }
}*/

//--------------------fonction d'acquisition d'etat d'un servo-----------------------
int8_t getStatus(uint8_t id, uint8_t numero_serial)
//
// renvoi l'état du servomoteur (doc pg 39)
//
//id ==> Id du servo concerné
//numero-serial ==> numéro de la liaison série du servo
//
{
    serial_numero = numero_serial;
    uint8_t status;
    uint8_t txBuf[7];
    size_reponse = 9;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE; // Packet Size
    txBuf[3] = id; // Servo ID
    txBuf[4] = CMD_STAT; // Status Error, Status Detail request
// Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    uint8_t rxBuf[9];

    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
                ////////////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }

// Checksum1
    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }
// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }
    status = rxBuf[7]; // Status Error
//status = rxBuf[8]; // Status Detail


    return status;
}
//------------------fonction pour lire la position actuelle-----------------------
int16_t getPos(uint8_t id, uint8_t numero_serial)
//
//renvoie la position d'un servo
//
//!!!ne pas oublier d'utiliser servo_interrupt_en();!!!
//
//id           ==> id d'un servomoteur
//numero_serial==> numéro de la liaison série du servo
//
{
    serial_numero = numero_serial;
    uint16_t position = 0;
    uint8_t txBuf[9];
    size_reponse = 13;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 2; // Packet Size
    txBuf[3] = id; // Servo ID
    txBuf[4] = CMD_RAM_READ; // Command Ram Read
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = RAM_CALIBRATED_POSITION; // Address
    txBuf[8] = BYTE2;
// Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    uint8_t rxBuf[13];

    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
                //pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }


// Checksum1
    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]^rxBuf[12]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        /*#ifdef HERKULEX_DEBUG
        pc->printf("Checksum1 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        /* #ifdef HERKULEX_DEBUG
        pc->printf("Checksum2 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }    position = ((rxBuf[10]&0x03)<<8) | rxBuf[9];


//}
    return position;
}
//---------------fonction d'acquis d'etat de couple d'un servo------------------
//Obtenir la valeur du couple d'un servo
int8_t Get_Torque(int8_t id, uint8_t numero_serial)
//
//id ==> id du servomoteur sur la liaison série
//numero_serial ==> numéro de la liaison série sur laquelle se trouve le servomoteur
//
{
    serial_numero = numero_serial;
    uint8_t txBuf[9];
    int8_t Tor = 0;

    uint8_t iv=0;
    for(iv=0; iv<20; iv++) {
        rx2[iv] = 0;
    }

    txBuf[0] = HEADER;                            // Packet Header (0xFF)
    txBuf[1] = HEADER;                            // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 2;               // Packet Size
    txBuf[3] = id;                                // Servo ID
    txBuf[4] = CMD_RAM_READ;                      // Command Ram Read
    txBuf[5] = 0;                                 // Checksum1
    txBuf[6] = 0;                                 // Checksum2
    txBuf[7] = RAM_TORQUE_CONTROL;
    txBuf[8] = BYTE1;                             // Length
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;//Checksum2
    txBuf[6] = (~txBuf[5])&0xFE;                  // CheckSum2

    //pc.printf(" Torque ");

    uint8_t rxBuf[12];

    if(numero_serial == 1) {
        //send packet (mbed -> herkulex)
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }

        //send packet (mbed -> herkulex)
        //uint8_t rxBuf[12];
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
                //////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }

    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        /*#ifdef HERKULEX_DEBUG
        pc->printf("Checksum1 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        /* #ifdef HERKULEX_DEBUG
        pc->printf("Checksum2 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }
        Tor = rxBuf[9];
    /* #ifdef HERKULEX_DEBUG
    pc->printf("position = %04X(%d)\n", position, position);
    #endif*/
//}
    return Tor;
}
//---------------fonction pour lire le temperature max pour un servo------------
//obtenir la valeur de température maximum tolérée par le servomoteur
int8_t Get_Temperature_MAX(int8_t id, uint8_t numero_serial)
//
//id ==> id du servomoteur sur la liaison série
//numero_serial ==> numéro de la liaison série sur laquelle se trouve le servomoteur
//
{
    serial_numero = numero_serial;
    uint8_t txBuf[9];
    int8_t tempeMAX = 0;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 2; // Packet Size
    txBuf[3] = id; // Servo ID
    txBuf[4] = CMD_RAM_READ; // Command Ram Read
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = RAM_MAX_TEMPERATURE;
    txBuf[8] = BYTE1; // Length
    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    //pc.printf(" tempeMAX ");

    uint8_t rxBuf[12];

    if(numero_serial == 1) {
        //send packet (mbed -> herkulex)
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }

        //send packet (mbed -> herkulex)
        //uint8_t rxBuf[12];
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
                //////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }

    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        /*#ifdef HERKULEX_DEBUG
        pc->printf("Checksum1 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        /* #ifdef HERKULEX_DEBUG
        pc->printf("Checksum2 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }
        tempeMAX = rxBuf[9];
    /* #ifdef HERKULEX_DEBUG
    pc->printf("position = %04X(%d)\n", position, position);
    #endif*/
//}
    return tempeMAX;
}
//--------fonction de controle de position pour deux servo(same playtime)-------
//permet de déplacer deux servomoteurs sur la même liaison série avec le même temps d'execution
void positionControl_Mul_ensemble(uint8_t id, uint16_t position, uint8_t playtime, uint8_t setLED,uint8_t id2, uint16_t position2, uint8_t setLED2, uint8_t numero_serial)
//
//id
//

{
    serial_numero = numero_serial;
    float tempo=0;
//if (position > 1023) return; //1002-21
    if (playtime > 254) return; //1-254 == 11.2ms-2.844sec.
    tempo=playtime*0.012;
    uint8_t txBuf[16];
    etat = pos;
    pos_position = position;
    pos_time = playtime;
    pos_led = setLED;
    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 9; // Packet Size
    //txBuf[3] = MAX_PID; // pID is total number of servos in the network (0 ~ 253)
    txBuf[3] = 254; // broadcast ID
    txBuf[4] = CMD_S_JOG; // Command S JOG (0x06)
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = playtime; // Playtime
    txBuf[8] = position & 0x00FF; // Position (LSB, Least Significant Bit)
    txBuf[9] =(position & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
    txBuf[10] = POS_MODE | setLED; // Pos Mode and LED on/off
    txBuf[11] = id; // Servo ID
    txBuf[12] = position2 & 0x00FF; // Position (LSB, Least Significant Bit)
    txBuf[13] =(position2 & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
    txBuf[14] = POS_MODE | setLED2; // Pos Mode and LED on/off
    txBuf[15] = id2; // Servo ID
// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]^txBuf[12]^txBuf[13]^txBuf[14]^txBuf[15]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;
// send packet (mbed -> herkulex)

    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 16 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 16 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 16 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 16 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 16 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }
    wait(tempo);
    wait_ms(TEMPO_R);
}
//-----fonction de controle de position pour deux servo(different playtime)-----  //a changer...
void positionControl_Mul_playtime_different(uint8_t id, uint16_t position, uint8_t playtime, uint8_t setLED,uint8_t id2, uint16_t position2, uint8_t playtime2, uint8_t setLED2, uint8_t numero_serial)
//
//permet de controler deux servomoteurs avec des temps d'execution différents
//
{
    serial_numero = numero_serial;
//if (position > 1023) return; //1002-21
    if (playtime > 254) playtime = 254; //return; //1-254 == 11.2ms-2.844sec.
    if(playtime>playtime2) {
        tempo=playtime*0.012;
    } else if(playtime<playtime2) {
        tempo=playtime2*0.012;
    }
    uint8_t txBuf[17];
    etat = pos;
    pos_position = position;
    pos_time = playtime;
    pos_led = setLED;
    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 9; // Packet Size
    //txBuf[3] = MAX_PID; // pID is total number of servos in the network (0 ~ 253)
    txBuf[3] = 254; // broadcast ID
    txBuf[4] = CMD_I_JOG; // Command I JOG
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = position & 0x00FF; // Position (LSB, Least Significant Bit)
    txBuf[8] =(position & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
    txBuf[9] = POS_MODE | setLED; // Pos Mode and LED on/off
    txBuf[10] = id; // Servo ID
    txBuf[11] = playtime; // Playtime
    txBuf[12] = position2 & 0x00FF; // Position (LSB, Least Significant Bit)
    txBuf[13] =(position2 & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
    txBuf[14] = POS_MODE | setLED2; // Pos Mode and LED on/off
    txBuf[15] = id2; // Servo ID
    txBuf[16] = playtime2; // Playtime
// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]^txBuf[12]^txBuf[13]^txBuf[14]^txBuf[15]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;
// send packet (mbed -> herkulex)
//txPacket(12, txBuf);

    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 17 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 17 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 17 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 17 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 17 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }

   wait_ms(TEMPO_R);
}
//-----fonction de controle de position pour plusieurs servo(same playtime)-----
void positionControl_Mul_ensemble_complex(uint8_t nb_servo, uint8_t playtime, uint8_t* data, uint16_t* pos, uint8_t numero_serial) // uint16_t position, uint8_t setLED, uint8_t id
//
//Permet de controler tout les servos de la même liaison série avec le même temps d'execution
//
//
{
    serial_numero = numero_serial;
    //float tempo=0;
    uint8_t taille = 0,i = 0,idata = 0, ipos = 0;
    //if (position > 1023) return; //1002-21
    if (playtime > 254) return; //1-254 == 11.2ms-2.844sec.
    //tempo=playtime*0.012;
    taille = 7 + 1 + 4 * nb_servo;
    nombre_servo = nb_servo;
    pos_time = playtime;
    uint8_t txBuf[taille];
    etat = pos_mul_complex;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 1 + 4 * nb_servo; // Packet Size
    //txBuf[3] = MAX_PID; // pID is total number of servos in the network (0 ~ 253)
    txBuf[3] = 254; // broadcast ID
    txBuf[4] = CMD_S_JOG; // Command S JOG (0x06)
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = playtime; // Playtime

    for(i=0; i<nb_servo; i++) {
        txBuf[8+i*4] = pos[ipos] & 0x00FF; // Position (LSB, Least Significant Bit)
        txBuf[9+i*4] =(pos[ipos] & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
        position_servo_mul[ipos] = pos[ipos];
        ipos++;
        txBuf[10+i*4] = POS_MODE | data[idata]; // Pos Mode and LED on/off
        data_servo_mul[idata] = data[idata];
        idata++;
        txBuf[11+i*4] = data[idata]; // Servo ID
        data_servo_mul[idata] = data[idata];
        idata++;
    }

// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7];

    for(i=1; i<(taille-7); i++) {
        txBuf[5]=txBuf[5]^txBuf[7+i];
    }
    txBuf[5] = txBuf[5]& 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

// send packet (mbed -> herkulex)
    if(numero_serial == 1) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }

    /*for(uint8_t i = 0; i < taille ; i++)
        {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }*/
    wait_ms(TEMPO_R);
}
//--fonction de controle de position pour plusieurs servo(different playtime)---
void positionControl_Mul_ensemble_different_complex(uint8_t nb_servo, uint8_t* data, uint16_t* pos, uint8_t numero_serial) // uint16_t position, uint8_t setLED, uint8_t id, uint8_t playtime
//
//Permet de controler tout les servos de la même liaison série avec un temps d'execution différent
//
{
    serial_numero = numero_serial;
    float tempo=0;
    uint8_t Max_playtime = 0;
    uint8_t taille = 0,i = 0,idata = 0, ipos = 0,iplay_time = 0;
    //if (position > 1023) return; //1002-21
    //if (playtime > 254) return; //1-254 == 11.2ms-2.844sec.

    for(iplay_time=0; iplay_time<nb_servo; iplay_time++) {
        if(Max_playtime<data[2+3*iplay_time]) {
            Max_playtime=data[2+3*iplay_time];
        }
    }
    tempo=Max_playtime*0.012;
    taille = 7 + 5 * nb_servo;
    nombre_servo = nb_servo;
    uint8_t txBuf[taille];
    etat = pos_mul_complex_different;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 5 * nb_servo; // Packet Size
    //txBuf[3] = MAX_PID; // pID is total number of servos in the network (0 ~ 253)
    txBuf[3] = 254; // broadcast ID
    txBuf[4] = CMD_I_JOG; // Command I JOG (0x06)
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2

    for(i=0; i<nb_servo; i++) {
        txBuf[7+i*5] = pos[ipos] & 0x00FF; // Position (LSB, Least Significant Bit)
        txBuf[8+i*5] =(pos[ipos] & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
        position_servo_mul_different[ipos] = pos[ipos];
        ipos++;
        txBuf[9+i*5] = POS_MODE | data[idata]; // Pos Mode and LED on/off
        data_servo_mul_different[idata] = data[idata];
        idata++;
        txBuf[10+i*5] = data[idata]; // Servo ID
        data_servo_mul_different[idata] = data[idata];
        idata++;
        txBuf[11+i*5] = data[idata]; // Playtime
        data_servo_mul_different[idata] = data[idata];
        idata++;
    }

// Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
// Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = txBuf[2]^txBuf[3]^txBuf[4];

    for(i=1; i<(taille-6); i++) {
        txBuf[5]=txBuf[5]^txBuf[6+i];
    }
    txBuf[5] = txBuf[5]& 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

// send packet (mbed -> herkulex)
    if(numero_serial == 1) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < taille ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }

    /*for(uint8_t i = 0; i < taille ; i++)
        {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }*/
    wait(tempo);
    wait_ms(TEMPO_R);
}
//---------------fonction pour lire la tension minimale pour un servo----------------
int8_t Get_Tension_MIN(int8_t id, uint8_t numero_serial)
{
    serial_numero = numero_serial;
    uint8_t txBuf[9];
    int8_t tensionMIN = 0;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 2; // Packet Size
    txBuf[3] = id; // Servo ID
    txBuf[4] = CMD_RAM_READ; // Command Ram Read
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = RAM_MIN_VOLTAGE;
    txBuf[8] = BYTE1; // Length
    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    //pc.printf(" tensionMIN ");

    uint8_t rxBuf[12];

    if(numero_serial == 1) {
        //send packet (mbed -> herkulex)
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }

        //send packet (mbed -> herkulex)
        //uint8_t rxBuf[12];
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
                //////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }

    /*for(uint8_t i = 0; i < 9 ; i++)
        {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }

    // send packet (mbed -> herkulex)
    uint8_t rxBuf[12];
    //wait_ms(3);
    wait_ms(TEMPO_R);
    if(flag_serial1_receive)
        {
            for(unsigned char i4=0;i4<Size_trame_serial1; i4++)
            {
                rxBuf[i4] = char_receive_serial1[i4];
                ////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }*/

    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        /*#ifdef HERKULEX_DEBUG
        pc->printf("Checksum1 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        /* #ifdef HERKULEX_DEBUG
        pc->printf("Checksum2 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }
        tensionMIN = rxBuf[9];
    /* #ifdef HERKULEX_DEBUG
    pc->printf("position = %04X(%d)\n", position, position);
    #endif*/
//}
    return tensionMIN;
}
//-------------fonction pour controle la tension min pour un servo--------------
void Set_Tension_MIN(int8_t id,uint8_t Tension_Min, uint8_t numero_serial)
{
    serial_numero = numero_serial;
    uint8_t txBuf[10];

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 3; // Packet Size
    txBuf[3] = id; // Servo ID
    txBuf[4] = CMD_RAM_WRITE; // Command Ram Write (0x03)
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = RAM_MIN_VOLTAGE;
    txBuf[8] = BYTE1; // Length
    txBuf[9] = Tension_Min;
    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    //pc.printf(" tensionMIN ");
    /*for(uint8_t i = 0; i < 10 ; i++)
        {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }*/
    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 10 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
    }
    //wait_ms(3);
    wait_ms(TEMPO_R);
}
//------------fonction pour lire la tension d'un servo-------------
int8_t Get_Tension_actuelle(int8_t id, uint8_t numero_serial)
{
    serial_numero = numero_serial;
    uint8_t txBuf[9];
    int8_t tension = 0;

    txBuf[0] = HEADER; // Packet Header (0xFF)
    txBuf[1] = HEADER; // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 2; // Packet Size
    txBuf[3] = id; // Servo ID
    txBuf[4] = CMD_RAM_READ; // Command Ram Read (0x03)
    txBuf[5] = 0; // Checksum1
    txBuf[6] = 0; // Checksum2
    txBuf[7] = RAM_VOLTAGE;
    txBuf[8] = BYTE2; // Length
    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    //pc.printf(" tension ");

    uint8_t rxBuf[13];

    if(numero_serial == 1) {
        //send packet (mbed -> herkulex)
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }

        //send packet (mbed -> herkulex)
        //uint8_t rxBuf[13];
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial1; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
                //////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial2; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
                //////pc.printf("%d ",(int)char_receive_serial2[i4]);
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial3; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
                //////pc.printf("%d ",(int)char_receive_serial3[i4]);
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial4; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
                //////pc.printf("%d ",(int)char_receive_serial4[i4]);
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 9 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<Size_trame_serial5; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
                //////pc.printf("%d ",(int)char_receive_serial5[i4]);
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }
    /*
    for(uint8_t i = 0; i < 9 ; i++)
        {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }

    // send packet (mbed -> herkulex)
    uint8_t rxBuf[13];
    //wait_ms(3);
    wait_ms(TEMPO_R);
    if(flag_serial1_receive)
        {
            for(unsigned char i4=0;i4<Size_trame_serial1; i4++)
            {
                rxBuf[i4] = char_receive_serial1[i4];
                ////pc.printf("%d ",(int)char_receive_serial1[i4]);
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    */

    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]^rxBuf[12]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        /*#ifdef HERKULEX_DEBUG
        pc->printf("Checksum1 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        /* #ifdef HERKULEX_DEBUG
        pc->printf("Checksum2 fault\n");
        #endif*/
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }    tension = rxBuf[9];
    /* #ifdef HERKULEX_DEBUG
    pc->printf("position = %04X(%d)\n", position, position);
    #endif*/
//}
    return tension;
}


//-----------------------------------------------------------------------------------------
void servo_interrupt_en(void)
{
    Interrupt1_en();
    Interrupt2_en();
    Interrupt3_en();
    Interrupt4_en();
    Interrupt5_en();
}

int8_t Get_Led(uint8_t id, uint8_t numero_serie)
{
        char numero_serial = numero_serie;
        uint8_t txBuf[7];
        txBuf[0] = HEADER;                                // Packet Header (0xFF)
        txBuf[1] = HEADER;                                // Packet Header (0xFF)
        txBuf[2] = 0x07;                                  // Packet Size
        txBuf[3] = id;                                    // Servo ID
        txBuf[4] = 0x05;                                  // Command Ram Write (0x03)   *On choisi le CMD pour écrire dans un registre
        txBuf[5] = 0;                                     // Checksum1
        txBuf[6] = 0;                                     // Checksum2
        
        // calcul des checksums //
        txBuf[5]=(txBuf[2]^txBuf[3]^txBuf[4]) & 0xFE;    // checksum1
        txBuf[5] &= 0xFE;   // checksum1
        txBuf[6] = (~txBuf[5] & 0xFE);//checksum2


        // envoi //
        //txPacket(0x07,txBuf,numero_serial);
        //reception 
    
    

    uint8_t rxBuf[9];

    if(numero_serial == 1) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial1.writeable() == 0);
            serial1.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial1_receive) {
            for(unsigned char i4=0; i4<8; i4++) {
                rxBuf[i4] = char_receive_serial1[i4];
            }
            flag_serial1_receive=0;
            valueserial1=0;
        }
    } else if(numero_serial == 2) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial2.writeable() == 0);
            serial2.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial2_receive) {
            for(unsigned char i4=0; i4<8; i4++) {
                rxBuf[i4] = char_receive_serial2[i4];
            }
            flag_serial2_receive=0;
            valueserial2=0;
        }
    } else if(numero_serial == 3) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial3.writeable() == 0);
            serial3.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial3_receive) {
            for(unsigned char i4=0; i4<8; i4++) {
                rxBuf[i4] = char_receive_serial3[i4];
            }
            flag_serial3_receive=0;
            valueserial3=0;
        }
    } else if(numero_serial == 4) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial4.writeable() == 0);
            serial4.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial4_receive) {
            for(unsigned char i4=0; i4<8; i4++) {
                rxBuf[i4] = char_receive_serial4[i4];
            }
            flag_serial4_receive=0;
            valueserial4=0;
        }
    } else if(numero_serial == 5) {
        for(uint8_t i = 0; i < 7 ; i++) {
            while(serial5.writeable() == 0);
            serial5.putc(txBuf[i]);
            wait_us(100);
        }
        wait_ms(TEMPO_R);
        if(flag_serial5_receive) {
            for(unsigned char i4=0; i4<8; i4++) {
                rxBuf[i4] = char_receive_serial5[i4];
            }
            flag_serial5_receive=0;
            valueserial5=0;
        }
    }

// Checksum1
    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]) & 0xFE;
    if (chksum1 != rxBuf[5]) {
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }// Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6]) {
        if(numero_serial == 1) {
            flag_serial1_receive=0;
        } else if(numero_serial == 2) {
            flag_serial2_receive=0;
        } else if(numero_serial == 3) {
            flag_serial3_receive=0;
        } else if(numero_serial == 4) {
            flag_serial4_receive=0;
        } else if(numero_serial == 5) {
            flag_serial5_receive=0;
        }
        return -1;
    }    int8_t couleur_hrk = rxBuf[7]; //  Couleur

    return couleur_hrk;
}