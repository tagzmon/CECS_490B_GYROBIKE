# include <stdlib.h>
# include <stdio.h>
# include <lpc214x.h>
# include <math.h>

void init_sys(void);
void init_pwm(void);
void init_timer0(void);
void init_timer1(void);
void init_ADC(void);
void init_SPI0(void);
void init_ACCSPI(void);
void init_timer0INTR(void);
void init_timer1INTR(void);
void read_SPI(void);
void process_sig(void);

void timer0ISR(void)__irq;
void timer1ISR(void)__irq;

unsigned int tickcount;
unsigned i;
unsigned j;
unsigned status;
unsigned readcam;
unsigned analogread;
unsigned pidcompute;
unsigned max_index;
unsigned setpoint_index;
unsigned readcount;

int min;
int max;
int threshcount;
int configure;
int testindex;

char values[10];
short SPI_DATAX;
short SPI_DATAY;
short SPI_DATAZ;

short aval;
short data[128];  //128 analog values
short* dataptr;

signed deriv[128];
unsigned* derivptr;
double  abdata[128];
double*  abdataptr;
unsigned testdata[100];
int main(void){
	
	 configure = 1;
	 readcount = 0;
	 tickcount = 0;
	 readcam = 0;
	 analogread = 0;
	 pidcompute = 0;
	
	 dataptr = data;
	 
	
	 init_sys();	 
	 init_pwm();
	 init_timer0();
	 init_timer1();
	 init_ADC();
	 init_SPI0();	 
	 init_timer0INTR();
	 init_timer1INTR();
	
   FIO0DIR = 0x12000080; //Servo PWM
	 FIO0MASK = 0x00000000;
	 FIO0PIN = 0x02000080; //initialize high
	 
	 
	 init_ACCSPI();
	 
	 //hard code camera setpoint
	 //setpoint_index = 68;
	 
	 PWMMR1 = 0x00016B3C;        //11% duty cycle - STRAIGHT
	 PWMLER = 0x02;              //Enable PWM1 Latch	 
	 //wait for a few seconds
	 for(i = 0; i < 10000000;i++){		 
	 }
	 
	 
   while(1){
      if(pidcompute == 1){				 
				 read_SPI();
//				 process_sig();
				
				 //find maximum of unprocessed signal
				 max_index = 20;  //first 18 readings are unpredictable
         for(i = 20; i < 128; i++){
				    if(data[i] > data[max_index]){
               max_index = i;     
            }							
				 }
				 
//				 testdata[testindex] = max_index;
//				 if(testindex == 99){
//					    derivptr = &testdata[0];
//					    testindex = 0;
//				 }
//				 testindex++;
				 
				 //configure camera setpoint
				 if(configure == 1){
				    setpoint_index = max_index;
            configure = 0;					 
				 }
				 else{
//					 derivptr = &deriv[0];
//					 abdataptr = &abdata[0];
					 
					 //TEST @ 10 * 25 ms = 250 ms
					 if(readcount == 10){
							readcount = 0; 
							if(max_index < setpoint_index-10){
								 PWMMR1 = 0x00010920;        //8% duty cycle - LEFT
								 PWMLER = 0x02;              //Enable PWM1 Latch
							}
							else if(max_index > setpoint_index+10){
								 PWMMR1 = 0x0001AA90;        //13% duty cycle - RIGHT
								 PWMLER = 0x02;              //Enable PWM1 Latch	 
							}
							else{
								 PWMMR1 = 0x00016B3C;        //11% duty cycle - STRAIGHT
								 PWMLER = 0x02;              //Enable PWM1 Latch	 
							}
					 }
					 readcount++;
			   }
				 
				 //wait for next read for PID computation
				 pidcompute = 0;			 	  
	      
      }
      			
//      for(i = 0; i < 1000000; i++){
//      }				
//	    PWMMR1 = 0x000189C0;         //12% DUTY CYCLE
//      PWMLER = 0x02;              //Enable PWM1 Latch
//      for(i = 0; i < 1000000; i++){
//      }				
//      PWMMR1 = 0x00010920;        //8% duty cycle - LEFT
//		  PWMLER = 0x02;              //Enable PWM1 Latch
       		 
   }  	 
}

//System initialization
void init_sys(void){
	 //PCLK = CCLK = 60 MHz
   VPBDIV = 0x00000001;
	
   // Pin Initialization
   // PWM1 Pin Selection - Servo
   // SPI SCK0
   // SPI MISO0
   // SPI MOSI0
   // SPI SSEL0	
   PINSEL0 |= 0x00001502; 
	 // GPIO P0.25 Pin Selection - Motor
	 // MAT0.3 Pin Selection - Camera Clk
	 // GPIO P0.28 Pin Selection - Camera SI
	 // AD0.3 Pin Selection - Camera AIN
   PINSEL1 |= 0x1C000000; 
	
	 //system control settings
	 //PORT 0 - bit 0: fast GPIO 
	 //PORT 1 - bit 1: slow GPIO 
	 SCS = 0x00000001;
	
	 //setup PLL
	 PLL0CFG = 0x24;
	 PLL0CON = 0x01;
	 PLL0FEED = 0xAA;
	 PLL0FEED = 0x55;
	 //wait until PLL is locked	 
	 do{
	    status = PLL0STAT;	
	 }while((status & 0x00000400) == 0);
	 //switch to PLL clock
	 PLL0CON = 0x03;
	 PLL0FEED = 0xAA;
	 PLL0FEED = 0x55;	 
	 
	 //setup MAM
	 MAMTIM = 0x00000003;
	 MAMCR = 0x00000002;  	
}

void init_pwm(void){
	 PWMPCR = 0x0000;
	 
   PWMPR =  0x0;
   PWMMR0 = 0x000CD140;        //14 ms signal
	 PWMMR1 = 0x00010920;        //8% duty cycle
   
   PWMMCR = 0x02;              //Reset counter on match	
	 PWMLER = 0x03;	             //Enable PWM 0 & 1 & 2 latch
   PWMPCR = 0x0200;            //Enable PWM1 & 2output   
	 
	 PWMTCR = 0x02;              //Counter Reset
	 PWMTCR = 0x09;              //Counter and PWM Enable
	
}
void init_timer0(void){
	
   T0MCR = 0x0601;	           //MR0 interrupt & MR3 interrupt and reset
   T0TCR = 0x01;               //enable counter
   T0MR0 = 0x00000400;         //20% duty cycle
	 T0MR3 = 0x00000BB8;         //20 kHz signal
	 T0EMR = 0x0C00;             //External Match 3 for Camera Clk
	
}
void init_timer1(void){
	 T1MCR = 0x0003;	           //MR0 interrupt and reset
   T1TCR = 0x01;               //enable counter
   T1MR0 = 0x0016E360;         //25 ms - 0x0016e360; 500 ms - 0x01c9c380
	  
}
void init_SPI0(void){
   S0SPCCR = 0x003C; // 60MHz/1MHz
	 //master mode 16 bit transfter with  CPHA and CPOL set to one.
	 S0SPCR = 0x03C;
}
void init_ACCSPI()
{	
	/*
	Need to initialize the POWER_CTL register (0x2D) to set the device
	into measurement mode by setting bit 3 to a 1 (0x08)
	
	Need to initialize the DATA_FORMAT register (0x31) to a g range.
	Typically, the smaller the range, the more sensitive the readings will be 
	from the accelerometer. Bits D0 and D1 set the Range where
	00 = 2g. 01 = 4g. 10 = 8g. 11 = 16g. For this register we set 0x01 for 4g.
	*/
	//Setting POWER_CTRL to 0x2D
	IO0CLR |= 0x00000080;	
	S0SPDR = 0x2D08;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	//Setting DATA_FORMAT to 0x01
	IO0CLR |= 0x00000080;	
	S0SPDR = 0x3101;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
}
void init_ADC(void){
   AD0CR = 0x00210608;
   AD0CR |= 0x01000000;
	 
}
void init_timer0INTR(void){
   VICVectCntl0 = 0x00000024;  //SLOT 0 -> timer0
   VICVectAddr0 = (unsigned long) timer0ISR;
   VICIntEnable |= 0x00000010; //enable timer0 interrupt
}
void init_timer1INTR(void){
   VICVectCntl1 = 0x00000025;  //SLOT 1 -> timer1
   VICVectAddr1 = (unsigned long) timer1ISR;
   VICIntEnable |= 0x00000020; //enable timer1 interrupt	
}
void read_SPI()
{
	/*
//Grab Lower X
	IO0CLR |= 0x00000080;	
	S0SPDR = 0xB700;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	SPI_DATAX = S0SPDR;
	
//Grab Lower Y
	IO0CLR |= 0x00000080;	
	S0SPDR = 0xB900;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	SPI_DATAY = S0SPDR;

//Grab Lower Z
	IO0CLR |= 0x00000080;	
	S0SPDR = 0xBB00;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	SPI_DATAZ = S0SPDR;	
	*/
	
	IO0CLR |= 0x00000080;	
	S0SPDR = 0xB200;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	values[0] = S0SPDR;
	
	
	IO0CLR |= 0x00000080;	
	S0SPDR = 0xB400;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	values[1] = S0SPDR;
	
	IO0CLR |= 0x00000080;	
	S0SPDR = 0xB600;
	while((S0SPSR&0xF8) != 0x80);
	IO0SET |=	0x00000080;
	values[2] = S0SPDR;
	
	
	SPI_DATAX = (short)values[0];
	SPI_DATAY = (short)values[2];
	SPI_DATAZ = (short)values[4];
	//SPI_DATAX = ((short)values[1]<<8)|(short)values[0];
	//SPI_DATAY = ((short)values[3]<<8)|(short)values[2];
  //SPI_DATAZ = ((short)values[5]<<8)|(short)values[4];
	//put_SPI_LCD();
}
void timer0ISR(void)__irq{	  
	 FIO0PIN = FIO0PIN ^ 0x02000000; //Pin 25 - motor
	 if(readcam == 1 && T0IR == 0x08){
	    FIO0PIN |= 0x10000000;       //SI HIGH -> start camera readout
      readcam = 0;
      analogread = 1;		 
	 }
	 else if(T0IR == 0x08){
		  FIO0PIN &= 0xEFFFFFFF;
		  if(analogread == 1){
			   if(tickcount%2 == 1){
            aval = AD0DR3;	         
	          *dataptr = aval;         //store ADC sample in SRAM
	          dataptr = dataptr + 1;
				 }
				 tickcount++;
         if(tickcount == 256){
			      analogread = 0;
            tickcount = 0;
            dataptr = &data[tickcount];			//reset pointer	
					  pidcompute = 1;
			   }				 
			}
		  
		  
	 }
	 T0IR = 0x09;                    //reset interrupt
   VICVectAddr = 0x00000000;   	
}
void timer1ISR(void)__irq{	 
	 readcam = 1;
	 T1IR = 0x01;                    //reset interrupt
	 VICVectAddr = 0x00000000;
}

void process_sig(void){
	 /*camera signal preprocessing
	   method: 
	   Take the normalized absolute value of the differential
	   of the input signal to find edges of the line.
	   Current issues with finding a valid threshold
	 */
   for(j = 0; j < 127; j++){
      deriv[j] = (signed)(data[j] - data[j+1]);
   }
	 deriv[127] = deriv[126]; //make copy of last derivative for minimal change
	 min = 0;
	 max = 0;
	 //produce absolute value
   for(j = 0; j < 128; j++){
      abdata[j] = fabs((double)deriv[j]);
		  if(abdata[j] < abdata[min]){
			   min = j;	
			}
			if(abdata[j] > abdata[max]){
			   max = j;	
			}
   }
	 threshcount = 0;
	 //normalize
	 for(j = 0; j < 128; j++){
	    abdata[j] = (abdata[j] - abdata[min])/(abdata[max] - abdata[min]);
		  if(abdata[j] > 0.8){
			   threshcount++;
			}
	 }
	
   	 
}



