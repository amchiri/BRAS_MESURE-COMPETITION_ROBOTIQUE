#include "main.h"

#define SIZE_FIFO 50


CAN can(PB_8,PB_9,1000000); // Rx&Tx pour le CAN



Serial pc(USBTX,USBRX,115200);

DigitalOut me(PA_4,1);
char data[2]= {0x75};
CANMessage tx_msg(0x732,data,1,CANData,CANStandard);
CANMessage msgRxBuffer[SIZE_FIFO];
unsigned char FIFO_ecriture=0; //Position du fifo pour la reception CAN
signed char FIFO_lecture=0;//Position du fifo de lecture des messages CAN
unsigned char EtatGameEnd=0, EtatGameStart = 0, EtatGameRecalage = 0;
unsigned short ackFinAction = 0;

AnalogIn avant_droit(PB_0);//1
AnalogIn avant_gauche(PB_1);//A
CANMessage tx_message;
char bidon[8];

char bras_choix=0,num_ca=0,msg_carre=0;
void initialisation_CAN (void) ;



void canProcessRx(void);

/*********************************************************************************************/
/* FUNCTION NAME: canRx_ISR                                                                  */
/* DESCRIPTION  : lit les messages sur le can et les stocke dans la FIFO                     */
/*********************************************************************************************/
void canRx_ISR (void)
{
    if (can.read(msgRxBuffer[FIFO_ecriture])) {
        FIFO_ecriture=(FIFO_ecriture+1)%SIZE_FIFO;
    }
}

int main()
{
    can.attach(&canRx_ISR); // création de l'interrupt attachée à la réception sur le CAN
    SendRawId(ALIVE_ACTIONNEURS_AVANT);
    CANMessage tx_message(0x222, bidon, 2, CANData, CANStandard);
    servo_interrupt_en(); //permettre les interuptions
    wait(1);
    deverouillage_torque();
    setTorque(101,TORQUE_ON,2);
    setTorque(4,TORQUE_ON,2);
    setTorque(102,TORQUE_ON,2);
    setTorque(5,TORQUE_ON,2);
    pc.printf("\nLAUNCHED\n\r");
    //on rentre tous les bras dans le robot dans le vagin de sa mere
//    gabarit_robot_gauche();
//    wait_ms(300);
//    gabarit_robot_droit();
//    wait_ms(300);
//    gabarit_robot_manche();
//    wait_ms(300);
    //DigitalIn cap2(PC_14);
//    SendRawId(CHECK_ACTIONNEURS_AVANT);
    while(1) {
        canProcessRx();
//        f_mesure();//dt35
        traitement();
//        selection_bras_attraper();
//        selection_bras_relacher();
//        automate_manche_air_haut();
//        automate_manche_air_bas();
//        automate_manche_air_moy();
//        selection_bras_prepa();
//        selection_bras_poser();
        /*
        automate_bras_attraper_1();
        automate_bras_relacher_1();
        automate_bras_attraper_2();
        automate_bras_relacher_2();
        automate_bras_attraper_3();
        automate_bras_relacher_3();
        */
        //automate_manche_air() ;
        /*automate_position_lidar();*/
        tx_message.data[1] = avant_droit.read() * 260;
        tx_message.data[0] = avant_gauche.read() * 260;
        can.write(tx_message);
        wait_ms(20);
    }
}
//fin du main

/****************************************************************************************/
/* FUNCTION NAME: canProcessRx                                                          */
/* DESCRIPTION  : Fonction de traitement des messages CAN                               */
/****************************************************************************************/
void canProcessRx(void)
{
    static signed char FIFO_occupation=0,FIFO_max_occupation=0;
    CANMessage msgTx=CANMessage();

    FIFO_occupation=FIFO_ecriture-FIFO_lecture;
    if(FIFO_occupation<0)
        FIFO_occupation=FIFO_occupation+SIZE_FIFO;

    if(FIFO_max_occupation<FIFO_occupation)
        FIFO_max_occupation=FIFO_occupation;

    if(FIFO_occupation!=0) {
        int identifiant=msgRxBuffer[FIFO_lecture].id;

        switch(identifiant) {
            case GLOBAL_START:
                EtatGameStart = 1;
                EtatGameEnd = 0;
                break;
            case RECALAGE_START :
                EtatGameRecalage = 1;
                break;

            case INSTRUCTION_END_MOTEUR:
                ackFinAction = msgRxBuffer[FIFO_lecture].data[0]|((unsigned short)(msgRxBuffer[FIFO_lecture].data[1])<<8);
                break;

            case GLOBAL_GAME_END:
                EtatGameEnd = 1;
                EtatGameStart = 0;
                break;

            case DATA_TELEMETRE: //Lit le telemetre N°X suivant la data dans le CAN
                char numero_telemetre=msgRxBuffer[FIFO_lecture].data[0];
                short distance=lecture_telemetre(numero_telemetre);


                msgTx.id=RECEPTION_DATA; // tx Valeur Telemetre1
                msgTx.len=2;
                msgTx.format=CANStandard;
                msgTx.type=CANData;
                // Rayon sur 2 octets
                msgTx.data[0]=(unsigned char)distance;
                msgTx.data[1]=(unsigned char)(distance>>8);
                can.write(msgTx);
                SendAck(ACKNOWLEDGE_TELEMETRE,RECEPTION_DATA);
                break;

            case DATA_RECALAGE:
                short distance1=lecture_telemetre(1);
                short distance2=lecture_telemetre(2);
                short distance3=lecture_telemetre(3);
                short distance4=lecture_telemetre(4);

                msgTx.id=RECEPTION_RECALAGE; // tx Valeur Telemetre1
                msgTx.len=8;
                msgTx.format=CANStandard;
                msgTx.type=CANData;
                // Rayon sur 2 octets
                msgTx.data[0]=(unsigned char)distance1;
                msgTx.data[1]=(unsigned char)(distance1>>8);
                msgTx.data[2]=(unsigned char)distance2;
                msgTx.data[3]=(unsigned char)(distance2>>8);
                msgTx.data[4]=(unsigned char)distance3;
                msgTx.data[5]=(unsigned char)(distance3>>8);
                msgTx.data[6]=(unsigned char)distance4;
                msgTx.data[7]=(unsigned char)(distance4>>8);
                can.write(msgTx);
                SendAck(ACKNOWLEDGE_TELEMETRE,RECEPTION_RECALAGE);
                break;

            case DATA_TELEMETRE_LOGIQUE:
                msgTx.id=RECEPTION_TELEMETRE_LOGIQUE; // tx Valeur Telemetre1
                msgTx.len=4;
                msgTx.format=CANStandard;
                msgTx.type=CANData;
                msgTx.data[0]=(unsigned char)DT1_interrupt_Ex;
                msgTx.data[1]=(unsigned char)DT2_interrupt_Ex;
                msgTx.data[2]=(unsigned char)DT3_interrupt_Ex;
                msgTx.data[3]=(unsigned char)DT4_interrupt_Ex;
                can.write(msgTx);
                SendAck(ACKNOWLEDGE_TELEMETRE,RECEPTION_TELEMETRE_LOGIQUE);
                break;

//----------------------------------------------------------------cases test-----------------------------------------------------------------//

            case TEST_BRAS_A:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_3_at = 0;
                break;

            case TEST_BRAS_B:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_3_re = 0;
                break;

            case TEST_BRAS_C:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_3_at = 1;
                break;

            case TEST_BRAS_D:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_3_re = 1;
                break;

            case TEST_BRAS_1:
                test_BRAS_1();
                break;

            case TEST_BRAS_2:
                test_BRAS_2();
                break;

            case TEST_BRAS_3:
                test_BRAS_3();
                break;

            case TEST_BRAS_4:
                test_BRAS_4();
                break;

            case TEST_BRAS_5:
                break;

            case TEST_BRAS_6:
                break;
////////////////////////////////////////////CASE DE canPETITIONS/////////////////////////////////////////////
            case BRAS_AT:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_at = 1;
                break;

            case BRAS_RE:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_re = 1;
                break;

            case BRAS_PREPA:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_prepa = 1;
                break;

            case BRAS_POSE:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_bras_av_pose = 1;
                break;

            case GABARIT_ROBOT:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                gabarit_robot_droit();
                gabarit_robot_gauche();
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_FIN_ACTION);
                break;

            case AUTOMATE_MANCHE_HAUT:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_manche_haut = 1;
                break;

            case AUTOMATE_MANCHE_BAS:
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_manche_bas = 1;
                break;

            case AUTOMATE_MANCHE_MOY :
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                bras_choix = msgRxBuffer[FIFO_lecture].data[0];
                aut_manche_moy = 1;
                break ;

            case PAVILLON_DEPLOYE :
                SendAck(ACKNOWLEDGE_HERKULEX, ACK_ACTION);
                pavilon_deploye();
                break ;

            case LECTURE_GIROUETTE :
                unsigned char port_final ;
                port_final = lecture_girouette() ;
                //if( port_final != 2)
                SendMsgCan(VALEUR_GIROUETTE, &port_final, sizeof(char)) ;
                break ;

            case TEST_LECTURE_GIROUETTE :
                test_lecture_girou() ;
                break ;
            case BF_RANGER:
                msg_carre=1;

                pc.printf(" ok recu ");
                break;

            case BF_PRETEST:
                msg_carre=3;


                break;

            case BF_MESURE:
                msg_carre=2;
                num_ca=msgRxBuffer[FIFO_lecture].data[0];

                break;

            case BF_COLOR:
                set_color(msgRxBuffer[FIFO_lecture].data[0] & 0x1);
                break;

            case BF_TEST:
                BF_test_mesure(msgRxBuffer[FIFO_lecture].data[0] & 0x1);

                break;
            case BF_POS:
                Interrupt2_en();
                pc.printf(" haut: %d    ",getPos(ID_HAUT,2));
                Interrupt2_en();
                pc.printf(" MILLIEU: %d     ",getPos(ID_MILLIEU,2));

                break;

            default:
                break;
        }
        FIFO_lecture=(FIFO_lecture+1)%SIZE_FIFO;
    }
}

void initialisation_CAN(void)
{
    CANMessage msg_init;
    msg_init.id = CHECK_ACTIONNEURS_AVANT;
    msg_init.len=1;
    msg_init.format=CANStandard;
    msg_init.type=CANData;
    msg_init.data[0]= 0 ;

    can.write(msg_init);
}