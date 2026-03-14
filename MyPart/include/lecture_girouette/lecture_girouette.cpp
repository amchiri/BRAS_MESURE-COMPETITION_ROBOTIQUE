#include "main.h"

DigitalIn captuer_haut_girou(PC_3) ;
DigitalIn captuer_bas_girou(PB_10) ;

// noir = 1 // blanc = 0

char lecture_girouette (void) 
{
    char val_cpt_haut, val_cpt_bas ;
    val_cpt_haut = captuer_haut_girou.read() ;
    val_cpt_bas = captuer_bas_girou.read() ;
    
    if(val_cpt_haut == 1 && val_cpt_bas == 0)
        return 1 ; //Nord
        
    else if (val_cpt_haut == 0 && val_cpt_bas == 1)
        return 0 ; //Sud
        
    else if(  val_cpt_haut == 1 && val_cpt_bas == 1)
        return 2 ; //On sait pas
    
    else if(  val_cpt_haut == 0 && val_cpt_bas == 0)
        return 3 ; //On sait pas
    else
        return 4 ;
}

void test_lecture_girou (void)
{
    unsigned char message_test_girou ;
    
    message_test_girou = captuer_bas_girou.read() | (captuer_haut_girou.read() << 1) ;
    SendMsgCan(LECTURE_GIROUETTE, &message_test_girou, sizeof(char) ) ;
}