/*******************************************************
This program was created by the
CodeWizardAVR V3.14 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 11/5/2025
Author  : 
Company : 
Comments: 


Chip type               : ATmega328P
Program type            : Application
AVR Core Clock frequency: 16.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#include <mega328p.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <delay.h> 
#include <stdint.h>

// Declare your global variables here
#define CSA_low()(PORTC &= ~(1 << 2)) 
#define CSA_high()(PORTC |= (1 << 2))
#define CSB_low()(PORTD &= ~(1 << 2)) 
#define CSB_high()(PORTD |= (1 << 2))

unsigned char temp_xlsb1,temp_lsb1,temp_msb1,press_xlsb1,press_lsb1,press_msb1;
unsigned char temp_xlsb2,temp_lsb2,temp_msb2,press_xlsb2,press_lsb2,press_msb2;
  
int32_t raw_T1,raw_P1,Temp1,Press1;	//awalnya Temp_whole dan float disini
int32_t raw_T2,raw_P2,Temp2,Press2;	//awalnya Temp_whole dan float disini

unsigned int Temp_wholeA,Temp_floatA,Press_highA,Press_lowA;
unsigned int Temp_wholeB,Temp_floatB,Press_highB,Press_lowB;

unsigned short dig_T1; 
short dig_T2, dig_T3; 
unsigned short dig_P1;
short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9; 
long t_fine;

unsigned short dig_T1_B; 
short dig_T2_B, dig_T3_B; 
unsigned short dig_P1_B;
short dig_P2_B, dig_P3_B, dig_P4_B, dig_P5_B, dig_P6_B, dig_P7_B, dig_P8_B, dig_P9_B; 
long t_fine_B; 

char DS; 
uint8_t id1; 
uint8_t id2;
unsigned char program = 0;


//Fungsi SPI
unsigned char spi_coms(unsigned char data,unsigned char RW) //RW 0 = write, 1 = read, 2 plain data yang penting DODR = 0
{
if (RW == 0)
{
data &= ~(1 << 7); // set bit7 ke 0 kalo write
}
else if (RW == 1)
{
data |= (1 << 7); // set bit7 ke 1 kalo read
}
SPDR = data;
while (!(SPSR & (1 << SPIF))); //tunggu kelar / SPIF set 
return SPDR;
}

//Sensor A
void compensation_init_A(void)
{
unsigned char buffer[24];
unsigned char i;
short dig[12];

CSB_high(); //SPIB Mati / Stop
CSA_low(); //SPIA start
spi_coms(0x88,1); //set pointer dan ready read 
for (i=0 ; i <= 23 ; i++)
{
buffer[i] = spi_coms(0xFF,2);
};
CSA_high(); //SPIA stop

for (i=0 ; i<23 ; i=i+2)
{
dig[i/2] = buffer[i] | (buffer[i+1] << 8);
};
dig_T1 = dig[0]; 
dig_T2 = dig[1]; 
dig_T3 = dig[2]; 
dig_P1 = dig[3]; 
dig_P2 = dig[4]; 
dig_P3 = dig[5]; 
dig_P4 = dig[6]; 
dig_P5 = dig[7]; 
dig_P6 = dig[8]; 
dig_P7 = dig[9]; 
dig_P8 = dig[10]; 
dig_P9 = dig[11];
}

int compensate_T_A(int32_t adc_T)
{
long var1, var2 , T;
var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14; t_fine = var1 + var2;
T = (t_fine * 5 + 128) >> 8; //masih harus dibagi 100 
return T;
}

uint32_t compensate_P_A(int32_t adc_P)
{
float var1, var2, p;

var1 = ((float)t_fine / 2.0f) - 64000.0f;
var2 = var1 * var1 * ((float)dig_P6) / 32768.0f; 
var2 = var2 + var1 * ((float)dig_P5) * 2.0f;
var2 = (var2 / 4.0f) + ((float)dig_P4 * 65536.0f);
var1 = (((float)dig_P3 * var1 * var1 / 524288.0f) + ((float)dig_P2 * var1)) / 524288.0f; 
var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1);
if (var1 == 0.0f)
return 0; //meninggal kalo dibagi 0

p = 1048576.0f - (float)adc_P;
p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
var1 = ((float)dig_P9) * p * p / 2147483648.0f; var2 = p * ((float)dig_P8) / 32768.0f;
p = p + (var1 + var2 + (float)dig_P7) / 16.0f; if (p < 0.0f) p = 0.0f;
if (p > 4294967295.0f) p = 4294967295.0f;
return (uint32_t)(p + 0.5f);
}



//Sensor B
void compensation_init_B(void)
{
unsigned char buffer[24];
unsigned char i;
short dig[12];

CSA_high(); //SPIB Mati / Stop
CSB_low(); //SPIA start
spi_coms(0x88,1); //set pointer dan ready read 
for (i=0 ; i <= 23 ; i++)
{
buffer[i] = spi_coms(0xFF,2);
};
CSB_high(); //SPIA stop

for (i=0 ; i<23 ; i=i+2)
{
dig[i/2] = buffer[i] | (buffer[i+1] << 8);
};
dig_T1_B = dig[0]; 
dig_T2_B = dig[1]; 
dig_T3_B = dig[2]; 
dig_P1_B = dig[3]; 
dig_P2_B = dig[4]; 
dig_P3_B = dig[5]; 
dig_P4_B = dig[6]; 
dig_P5_B = dig[7]; 
dig_P6_B = dig[8]; 
dig_P7_B = dig[9]; 
dig_P8_B = dig[10]; 
dig_P9_B = dig[11];
}

int compensate_T_B(int32_t adc_T)
{
long var1, var2 , T;
var1 = ((((adc_T >> 3) - ((int32_t)dig_T1_B << 1))) * ((int32_t)dig_T2_B)) >> 11;
var2 = (((((adc_T >> 4) - ((int32_t)dig_T1_B)) * ((adc_T >> 4) - ((int32_t)dig_T1_B))) >> 12) *((int32_t)dig_T3_B)) >> 14; t_fine_B = var1 + var2;
T = (t_fine_B * 5 + 128) >> 8; //masih harus dibagi 100 
return T;
}

uint32_t compensate_P_B(int32_t adc_P)
{
float var1, var2, p;

var1 = ((float)t_fine_B / 2.0f) - 64000.0f;
var2 = var1 * var1 * ((float)dig_P6_B) / 32768.0f; 
var2 = var2 + var1 * ((float)dig_P5_B) * 2.0f;
var2 = (var2 / 4.0f) + ((float)dig_P4_B * 65536.0f);
var1 = (((float)dig_P3_B * var1 * var1 / 524288.0f) + ((float)dig_P2_B * var1)) / 524288.0f; 
var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1_B);
if (var1 == 0.0f)
return 0; //meninggal kalo dibagi 0

p = 1048576.0f - (float)adc_P;
p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
var1 = ((float)dig_P9_B) * p * p / 2147483648.0f; var2 = p * ((float)dig_P8_B) / 32768.0f;
p = p + (var1 + var2 + (float)dig_P7_B) / 16.0f; if (p < 0.0f) p = 0.0f;
if (p > 4294967295.0f) p = 4294967295.0f;
return (uint32_t)(p + 0.5f);
}

void compensation_init_whole(void) {
    compensation_init_A();
    compensation_init_B();
}


interrupt [USART_RXC] void recieve_isr(void)
{
PORTC.3 = 1;
DS = UDR0; 
if(DS== '$')
{
    if(program == 0)
    {
    program = 2;
    }
    else
    {
    program = 0;
    }
}

else if(DS== '*')
    {
    if(program!=0)
        {
            program=1;
        }
    };    
}

void main(void)
{
// Declare your local variables here

// Crystal Oscillator division factor: 1 
#pragma optsize- 
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_ 
#pragma optsize+ 
#endif

DDRD = 0x06; //PD2 buat chip select
PORTD.2 = 1; //Select chip selectnya harus high buat off
DDRB |= (1 << 3) | (1 << 5); // MOSI (PB3), SCK (PB5) jadi output janlup... scknya searah 
DDRB &= ~(1 << 4);	// MISO PB4 input
PORTB &= ~(1 << 4);	// no pull-up on MISO

DDRC = 0x0C;  //PD2 buat chip select
PORTC.2 = 1;  //Select chip selectnya harus high buat off
PORTC.3 = 0; // Indikator Interrupt

UBRR0H = 0x00; UBRR0L = 0x67;
UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << 7);
UCSR0C = (1 << 1) | (1 << 2);

SPSR &= ~(1 << SPI2X); //Matiin SPI 2x mode
SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1); // sck_freq = 250khz, Master mode 
SPCR &= ~(1 << DORD); // MSB first

compensation_init_whole();

#asm("SEI");
while (1)
{
// Place your code here       
    if(program == 1)
    {
//-----------------------Sensor A----------------------  
      CSB_high();
        CSA_low();
        spi_coms(0xD0, 1); //pointe rke register ID
         
        id1 = spi_coms(0xFF, 2);
        CSA_high();
        
        CSA_low();
        spi_coms(0xF4,0); //pointer to ctrl meas register 
        spi_coms(0b00100101,0); //oversampling x1 temp and press, mode forced
        CSA_high();
        delay_ms(40);
        
        CSA_low();
        spi_coms(0xF7,1); //pointer to press_msb 
        press_msb1 = spi_coms(0xFF,2); //baca semua data 
        press_lsb1 = spi_coms(0xFF,2);
        press_xlsb1 = spi_coms(0xFF,2); 
        temp_msb1 = spi_coms(0xFF,2); 
        temp_lsb1 = spi_coms(0xFF,2); 
        temp_xlsb1 = spi_coms(0xFF,2); 
        CSA_high();
        
        raw_P1 = ((int32_t)press_msb1 << 12) | ((int32_t)press_lsb1 << 4) | ((int32_t)press_xlsb1 >> 4); //Convert to the 20bit data 
        raw_T1 = ((int32_t)temp_msb1 << 12) | ((int32_t)temp_lsb1 << 4) | ((int32_t)temp_xlsb1 >> 4);
        Temp1 = compensate_T_A(raw_T1); 
        Press1 = compensate_P_A(raw_P1); 
        Temp_wholeA = Temp1 / 100; 
        Temp_floatA = Temp1 % 100;
        Press_highA = (unsigned int)(Press1 / 100); 
        Press_lowA = (unsigned int)(Press1 % 100);
        if (Temp_floatA < 0) Temp_floatA = -Temp_floatA; 
        Temp_floatA = Temp_floatA - 2; //Nge0in diproteus 
        Press_lowA = Press_lowA - 3; //Nge0in diproteus
        

    //------------------------Sensor B--------------------
        CSB_low();
        spi_coms(0xD0, 1); //pointe rke register ID
         
        id2 = spi_coms(0xFF, 2);
        CSB_high();
        
        CSB_low();
        spi_coms(0xF4,0); //pointer to ctrl meas register 
        spi_coms(0b00100101,0); //oversampling x1 temp and press, mode forced
        CSB_high();
        delay_ms(40);
        
        CSB_low();
        spi_coms(0xF7,1); //pointer to press_msb 
        press_msb2 = spi_coms(0xFF,2); //baca semua data 
        press_lsb2 = spi_coms(0xFF,2);
        press_xlsb2 = spi_coms(0xFF,2); 
        temp_msb2 = spi_coms(0xFF,2); 
        temp_lsb2 = spi_coms(0xFF,2); 
        temp_xlsb2 = spi_coms(0xFF,2); 
        CSB_high();
        
        raw_P2 = ((int32_t)press_msb2 << 12) | ((int32_t)press_lsb2 << 4) | ((int32_t)press_xlsb2 >> 4); //Convert to the 20bit data 
        raw_T2 = ((int32_t)temp_msb2 << 12) | ((int32_t)temp_lsb2 << 4) | ((int32_t)temp_xlsb2 >> 4);
        Temp2 = compensate_T_B(raw_T2); 
        Press2 = compensate_P_B(raw_P2); 
        Temp_wholeB = Temp2 / 100; 
        Temp_floatB = Temp2 % 100;
        Press_highB = (unsigned int)(Press2 / 100); 
        Press_lowB = (unsigned int)(Press2 % 100);
        if (Temp_floatB < 0) Temp_floatB = -Temp_floatB; 
        Temp_floatB = Temp_floatB - 2; //Nge0in diproteus 
        Press_lowB = Press_lowB - 3; //Nge0in diproteus
        
        
        //printf("Temperature Sensor 1 = %d.%d C\n\r",Temp_wholeA,Temp_floatA); 
        //printf("Pressure Sensor 1 = %u.%02u hPa\n\r",Press_highA,Press_lowA);
        
        //printf("Temperature Sensor 2 = %d.%d C\n\r",Temp_wholeB,Temp_floatB); 
        //printf("Pressure Sensor 2 = %u.%02u hPa\n\r",Press_highB,Press_lowB);
        
        printf("%d.%d:%d.%d:%u.%02u:%u.%02u \n\r", Temp_wholeA,Temp_floatA , 
        Temp_wholeB,Temp_floatB , Press_highA,Press_lowA , Press_highB,Press_lowB);
        
        };

}
}
