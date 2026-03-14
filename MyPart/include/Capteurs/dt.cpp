#include "dt.h"
//RawSerial pc(USBTX, USBRX,9600);//serie pour debug

AnalogIn DT1(PB_0);
AnalogIn DT2(PC_5);
AnalogIn DT3(PC_4);
AnalogIn DT4(PA_5);

InterruptIn DT1_isr(PB_1);
InterruptIn DT2_isr(PA_7);
InterruptIn DT3_isr(PA_6);
InterruptIn DT4_isr(PA_4);

Timer t;

double DT1_mes[256]= {0};
double DT1_trait[256]= {0};
double DT1_trait_Ex;

double DT2_mes[256]= {0};
double DT2_trait[256]= {0};
double DT2_trait_Ex;

double DT3_mes[256]= {0};
double DT3_trait[256]= {0};
double DT3_trait_Ex;

double DT4_mes[256]= {0};
double DT4_trait[256]= {0};
double DT4_trait_Ex;

unsigned char DT1_interrupt_Ex;
unsigned char DT2_interrupt_Ex;
unsigned char DT3_interrupt_Ex;
unsigned char DT4_interrupt_Ex;

unsigned char n = 0;
bool flag_t=0;

void f_mesure()
{
    if(!flag_t) {
        t.start();
        flag_t=1;
    }

    if (t.read_ms() >= 4) {
        n++;
        DT1.read();//lecture dans le vide le temp que l'adc switch
        wait_us(100);//attente du switch de l'adc
        DT1_mes[n]= DT1.read() * A + B;
        DT1_trait[n] = mediane(DT1_mes, 10);
        DT1_trait_Ex = DT1_trait[n];

        DT2.read();//lecture dans le vide le temp que l'adc switch
        wait_us(100);//attente du switch de l'adc
        DT2_mes[n]= DT2.read() * A + B;
        DT2_trait[n] = mediane(DT2_mes, 10);
        DT2_trait_Ex = DT2_trait[n];

        DT3.read();//lecture dans le vide le temp que l'adc switch
        wait_us(100);//attente du switch de l'adc
        DT3_mes[n]= DT3.read() * A + B;
        DT3_trait[n] = mediane(DT3_mes, 10);
        DT3_trait_Ex = DT3_trait[n];

        DT4.read();//lecture dans le vide le temp que l'adc switch
        wait_us(100);//attente du switch de l'adc
        DT4_mes[n]= DT4.read() * A + B;
        DT4_trait[n] = mediane(DT4_mes, 10);
        DT4_trait_Ex = DT4_trait[n];

        t.reset();
    }

}

double mediane(double* buff_med, int size_med)
{
    double DT_med[30]= {0};
    for(unsigned char i =0; i< size_med; i++)DT_med[i] = buff_med[(unsigned char)(n-i)];

    tri(DT_med, size_med);
    return DT_med[(int)(size_med/2)];
}

void tri(double* tab, int size)
{
    for (int i=0; i<size; i++) {
        for(int j=i; j<size; j++) {
            if(tab[j]<tab[i]) {
                double temp = tab[i];
                tab[i] = tab[j];
                tab[j] = temp;
            }
        }
    }
}

void interrupt()
{
    DT1_interrupt_Ex = DT1_isr.read();
    DT2_interrupt_Ex = DT2_isr.read();
    DT3_interrupt_Ex = DT3_isr.read();
    DT4_interrupt_Ex = DT4_isr.read();
}