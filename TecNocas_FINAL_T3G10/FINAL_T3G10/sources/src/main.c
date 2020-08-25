#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "serial_printf.h"
#include "i2c.h"
#include "lcd1602.h"

//colours
#define BLACK 100
#define WHITE 920
#define GRAY 512

//encoders
#define ENC1_A 0 // B (D8)
#define ENC1_B 7 // D (D7)
#define ENC2_A 4 // B (D12)
#define ENC2_B 3 // B (D11)

//motores
#define AIN1 2 // B 
#define AIN2 1 // B
#define BIN1 4 // D
#define BIN2 3 // D
#define PWMA 5 // D
#define PWMB 6 // D

//sensores IV
#define IR1 0 // C
#define IR2 1 // C
#define IR3 2 // C
#define IR4 3 // C
#define IR5 6 // C
#define IR_REMOTE 7 // C
#define IR_D 16 
#define HALFLINE 9

//Touchswitch
#define TS 5 //B (D13)

//velocidades
#define BASEA 0 //entre 0 e 256
#define BASEB 0 //entre 0 e 256
#define SPEED_MIN 0
#define SPEED_MAX 40
#define VBASE 20

//timer enconders
#define BOTTOM 65436

//laps number
#define NLAPS 5

uint8_t count = 4, laps=0;
uint16_t sensx = 0;
uint8_t encA_1 = 0;
uint8_t encB_1 = 0;
uint8_t encA_2 = 0;
uint8_t encB_2 = 0;
uint32_t enc1[2]={0,0}, enc2[2]={0,0}, st_motorA=0, posA=0, st_motorB=0, posB=0, deltax=0, count_int=0, count_c=0,count_laps=0;

char lcd_message1[16], lcd_message2[16];

int32_t pos;
int32_t prop, der, old_prop, error;

int st_ts=0, old_ts=0, ts, state_line=0, state_robot=0;
uint8_t EEMEM laps_eeprom;

void init_interrupts(void)
{
    //enable all interrupts
    SREG = 0b10000000;
}

void init_IO(void)
{
    //set IR sensors pins as inputs
    DDRC &= ~(1<<IR1);
    DDRC &= ~(1<<IR2);
    DDRC &= ~(1<<IR3);
    DDRC &= ~(1<<IR4);
    DDRC &= ~(1<<IR5);
    DDRC &= ~(1<<IR_REMOTE);

    //set encoder pins as inputs
    DDRD &= ~(1<<ENC1_B);
    DDRB &= ~((1<<ENC1_A) | (1<<ENC2_A) | (1<<ENC2_B));

    //activate pull-up resistor for encoders
    PORTB |= (1<<ENC1_A) | (1<<ENC2_A) | (1<<ENC2_B);
    PORTD |= (1<<ENC1_B);

    //set motor pins
    DDRD |= (1<<PWMA) | (1<<PWMB);
    DDRB |= (1<<AIN1) | (1<<AIN2);
    DDRD |= (1<<BIN1) | (1<<BIN2);

    //set direction
    PORTB |= (1<<AIN1);
    PORTD |= (1<<BIN1);
    PORTB &= ~(1<<AIN2);
    PORTD &= ~(1<<BIN2);
    PORTD |= (1<<PWMA) | (1<<PWMB);

    //set touchswitch pin as input
    DDRB &= ~(1<<TS);
    PORTB |= (1<<TS);

}

//***************************** ANALOG READ ************************************
void read_analog(uint8_t bit)
{
    ADMUX = bit; //indicates which pin to read from
    ADMUX = ADMUX | (1<<REFS0);
    ADCSRA = ADCSRA | (1<<ADSC);
}

void init_analog(void)
{
    //define Vref=Vcc
    ADMUX = ADMUX | (1<<REFS0);

    //deactivate digital buffer at PC0
    DIDR0 = DIDR0 | 0b00011111;

    //prescaler = 128 and activate ADC
    ADCSRA |= (7<<ADPS0) | (1<<ADEN) | (1<<ADIE);

    //start reading
    read_analog(count);
}

ISR(ADC_vect)
{
    sensx = ADC;
    count++;

    //read pins 1 to 3
    if(count<4)
    {
        read_analog(count);
    }

    //read pin 6
    else if(count==4)
    {
        read_analog(6); //IR 5 is at pin 6
    }

    //read pin 7
    else if(count==5)
    {
        read_analog(7); //IR for remote control is at pin 7
    }

    //restart at pin 0
    else
    {
        count = 0;
        read_analog(count);
    }
}
//******************************************************************************

//******************************** PWM *******************************************
void init_pwm(void)
{
    TCCR0B = 0;
    TIFR0 |= (7<<TOV0);
    TCCR0A |= (1<<WGM00) | (1<<WGM01) | (1<<COM0A1) | (1<<COM0B1);
    TCNT0 = 0;
    OCR0A = BASEA;
    OCR0B = BASEB;
    TIMSK0 = 0;
    TCCR0B |= (1<<CS01) | (1<<CS00); //prescaler 64
}

void set_speed_A(float v)
{
    float aux;
    aux = v*255/100; //convert from percentage to 256 scale
    OCR0A = aux;
}

void set_speed_B(float v)
{
    float aux;
    aux = v*255/100; //convert from percentage to 256 scale
    OCR0B = aux;
}


//******************************************************************************

//*************************** FOLLOW LINE **************************************
void follow_line_left(int *IR_sensors)
{
    int32_t hbig, hsmall, dsmall;

    //calculate error based on 2 sensors
    hbig = IR_sensors[1] - IR_sensors[2] - 100;

    if(hbig<0)
        hbig=-hbig;

    hsmall = IR_sensors[1] - GRAY - 100;

    dsmall = hsmall*IR_D/hbig;

    error = dsmall - (IR_D - HALFLINE);

    //stop if completely out of the circuit
    if(IR_sensors[0]>=WHITE && IR_sensors[1]>=WHITE && IR_sensors[2]>=WHITE && IR_sensors[3]>=WHITE && IR_sensors[4]>=WHITE)
    {
        set_speed_A(0);
        set_speed_B(0);
    }

    //count a lap
    else if(IR_sensors[0]<=BLACK && IR_sensors[4]<=BLACK)
    {
        if(count_laps==0)
        {
            laps++;
            eeprom_update_byte(&laps_eeprom, laps);
            count_laps=40000;
        }

        set_speed_A(VBASE);
        set_speed_B(VBASE);
    }
    
    //set speed based on error
    else
    {
        if(VBASE-error<0)
            set_speed_A(0);
        else if(VBASE-error>100)
            set_speed_A(100);
        else
            set_speed_A(VBASE-error);

        if(VBASE+error<0)
            set_speed_B(0);

        else if(VBASE+error>100)
            set_speed_B(100);

        else
            set_speed_B(VBASE+error);
    }
}

void follow_line_right(int *IR_sensors)
{
    int32_t hbig, hsmall, dsmall;

    //calculate error based on 2 sensors
    hbig = IR_sensors[3] - IR_sensors[2] - 100;

    if(hbig<0)
        hbig=-hbig;

    hsmall = IR_sensors[3] - GRAY - 100;

    dsmall = hsmall*IR_D/hbig;

    error = dsmall - (IR_D - HALFLINE);

    //stop if completely out of the circuit
    if(IR_sensors[0]>=WHITE && IR_sensors[1]>=WHITE && IR_sensors[2]>=WHITE && IR_sensors[3]>=WHITE && IR_sensors[4]>=WHITE)
    {
        set_speed_A(0);
        set_speed_B(0);
    }

    //count a lap
    else if(IR_sensors[0]<=BLACK && IR_sensors[4]<=BLACK)
    {
        if(count_laps==0)
        {
            laps++;
            eeprom_update_byte(&laps_eeprom, laps);
            count_laps=40000;
        }

        set_speed_A(VBASE);
        set_speed_B(VBASE);
    }
    
    //set speed based on error
    else
    {
        if(VBASE+error<SPEED_MIN)
            set_speed_A(SPEED_MIN);
        else if(VBASE+error>SPEED_MAX)
            set_speed_A(SPEED_MAX);
        else
            set_speed_A(VBASE+error);

        if(VBASE-error<SPEED_MIN)
            set_speed_B(SPEED_MIN);

        else if(VBASE-error>SPEED_MAX)
            set_speed_B(SPEED_MAX);

        else
            set_speed_B(VBASE-error);
    }
}

//*******************************************************************************

//***************************** SENSORES ****************************************
void read_sensors(int *IR_sensors)
{
    //update IR sensors information
    cli();
    if(count>0 && count<6)
    {            
        IR_sensors[count-1] = sensx;
    }
    
    else if(count==0)
    {
        IR_sensors[5] = sensx;
    }
    sei();
}
//*******************************************************************************

//************************** ENCODERS E LCD *************************************
void init_timer1(void) //0.05 ms
{
    TCCR1B = 0; // Stop tc1
    TIFR1 |= (7<<OCF1A)|(1<<ICF1); // Clear interruptions
    TCCR1A = 0; // normal mode
    TCNT1 = BOTTOM; // Load BOTTOM value
    TIMSK1 = (1<<TOIE1); // Enable overflow interrupt
    TCCR1B = 2; // Start TC1 (TP=8)
}

ISR(TIMER1_OVF_vect)
{
    count_int++; //count time for lcd update (250 ms)

    //time to prevent counting more than 1 lap at a time
    if(count_c!=0)
        count_c--;

    if(count_laps!=0)
        count_laps--;

    //update encoders information
    if((PINB & (1<<0)) == 0)
        enc1[0] = 0;
    else
        enc1[0] = 1;
    if((PIND & (1<<7)) == 0)
        enc2[0] = 0;
    else
        enc2[0] = 1;
    if((PINB & (1<<4)) == 0)
        enc1[1] = 0;
    else
        enc1[1] = 1;
    if((PINB & (1<<3)) == 0)
        enc2[1] = 0;
    else
        enc2[1] = 1;

    //update motor A state
    if(st_motorA==0 && enc1[0]==1 && enc2[0]==0)
    {
        st_motorA=2;
        posA++;
    }
    else if(st_motorA==0 && enc1[0]==0 && enc2[0]==1)
    {
        st_motorA=1;
        posA--;
    }
    else if(st_motorA==2 && enc1[0]==1 && enc2[0]==1)
    {
        st_motorA=3;
        posA++;
    }
    else if(st_motorA==2 && enc1[0]==0 && enc2[0]==0)
    {
        st_motorA=0;
        posA--;
    }
    else if(st_motorA==1 && enc1[0]==1 && enc2[0]==1)
    {
        st_motorA=3;
        posA--;
    }
    else if(st_motorA==1 && enc1[0]==0 && enc2[0]==0)
    {
        st_motorA=0;
        posA++;
    }
    else if(st_motorA==3 && enc1[0]==0 && enc2[0]==1)
    {
        st_motorA=1;
        posA++;
    }
    else if(st_motorA==3 && enc1[0]==1 && enc2[0]==0)
    {
        st_motorA=2;
        posA--;
    }

    //update motor B state
    if(st_motorB==0 && enc1[1]==1 && enc2[1]==0)
    {
        st_motorB=2;
        posB--;
    }
    else if(st_motorB==0 && enc1[1]==0 && enc2[1]==1)
    {
        st_motorB=1;
        posB++;
    }
    else if(st_motorB==2 && enc1[1]==1 && enc2[1]==1)
    {
        st_motorB=3;
        posB--;
    }
    else if(st_motorB==2 && enc1[1]==0 && enc2[1]==0)
    {
        st_motorB=0;
        posB++;
    }
    else if(st_motorB==1 && enc1[1]==1 && enc2[1]==1)
    {
        st_motorB=3;
        posB++;
    }
    else if(st_motorB==1 && enc1[1]==0 && enc2[1]==0)
    {
        st_motorB=0;
        posB--;
    }
    else if(st_motorB==3 && enc1[1]==0 && enc2[1]==1)
    {
        st_motorB=1;
        posB--;
    }
    else if(st_motorB==3 && enc1[1]==1 && enc2[1]==0)
    {
        st_motorB=2;
        posB++;
    }
    TCNT1 = BOTTOM;                 // Load BOTTOM value (only count 800)
    
    //count centimeters moved
    if((posA+posB)/2>=210)
    {
        posA=0;
        posB=0;
        deltax++;
    }

    //lcd update
    if(count_int==5000)
    {
        count_int=0;
        char aux[10];

        if(state_robot==0 && laps<NLAPS && st_ts==1)
        {
            lcd1602_clear();
            strcpy(lcd_message1, "laps: ");
            itoa(laps, aux, 10);
            strcat(lcd_message1, aux);
            strcpy(lcd_message2, "moved: ");
            itoa(deltax, aux, 10);
            strcat(lcd_message2, aux);
            strcat(lcd_message2, "cm");
            lcd1602_goto_xy(0,0);
            lcd1602_send_string(lcd_message1);
            lcd1602_goto_xy(15,0);
            lcd1602_send_char('R');
            lcd1602_goto_xy(0,1);
            lcd1602_send_string(lcd_message2);
        }

        else if(state_robot==1 && laps<NLAPS && st_ts==1)
        {
            lcd1602_clear();
            strcpy(lcd_message1, "laps: ");
            itoa(laps, aux, 10);
            strcat(lcd_message1, aux);
            strcpy(lcd_message2, "moved: ");
            itoa(deltax, aux, 10);
            strcat(lcd_message2, aux);
            strcat(lcd_message2, "cm");
            lcd1602_goto_xy(0,0);
            lcd1602_send_string(lcd_message1);
            lcd1602_goto_xy(15,0);
            lcd1602_send_char('L');
            lcd1602_goto_xy(0,1);
            lcd1602_send_string(lcd_message2);
        }

        else if(laps>=NLAPS && st_ts==1)
        {
            lcd1602_clear();
            strcpy(lcd_message1, "DONE ");
            strcpy(lcd_message2, "moved: ");
            itoa(deltax, aux, 10);
            strcat(lcd_message2, aux);
            strcat(lcd_message2, "cm");
            lcd1602_goto_xy(0,0);
            lcd1602_send_string(lcd_message1);
            lcd1602_goto_xy(0,1);
            lcd1602_send_string(lcd_message2);
        }

        else
        {
            lcd1602_clear();
            lcd1602_goto_xy(0,0);
            lcd1602_send_string(" ACTIVATE WITH ");
            lcd1602_goto_xy(0,1);
            lcd1602_send_string("  MICROSWITCH  ");
        }
    }

}
//*******************************************************************************

int main(void)
{
    int IR_sensors[6];

    //initialize everything
    init_interrupts();
    init_IO();
    init_analog();
    printf_init();
    init_pwm();
    init_timer1();
    i2c_init();
    lcd1602_init();
    lcd1602_clear();

    //check if in the middle of a run
    laps = eeprom_read_byte(&laps_eeprom);
    if(laps<0 || laps>NLAPS)
    {
        eeprom_update_byte(&laps_eeprom, 0);
        laps = 0;
    }
    
    while(1)
    {
        read_sensors(IR_sensors);

        //MICROSWITCH CONTROL FOR ON AND OFF

        if(PINB & (1<<TS))
            ts=1;
        else
            ts=0;

        if(ts==1 && old_ts==0 && st_ts==0 && (laps==0 || laps==NLAPS))
        {
            st_ts=1;
            laps=0;
            deltax=0;
        }
        else if(ts==1 && old_ts==0 && st_ts==0 && (laps!=0 && laps!=NLAPS))
        {
            st_ts=1;
        }
        else if(ts==1 && old_ts==0 && st_ts==1)
        {
            st_ts=0;
            laps=0;
        }

        old_ts=ts;

        //REMOTE CONTROL FOR LINE CHOICE

        if(state_robot==0 && count_c==0 && IR_sensors[5]==0)
        {
            state_robot=1;
            count_c=5000;
        }
        else if(state_robot==1 && count_c==0 && IR_sensors[5]==0)
        {
            state_robot=0;
            count_c=5000;
        }

        //FOLLOW_LINE CHOICE

        if(laps<NLAPS && state_robot==0 && st_ts==1)
        {
            follow_line_right(IR_sensors);
        }
        else if(laps<NLAPS && state_robot==1 && st_ts==1)
        {
            follow_line_left(IR_sensors);
        }
        else
        {
            set_speed_A(0);
            set_speed_B(0);
        }
    }
}  