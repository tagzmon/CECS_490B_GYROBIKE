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
void configure(void);
void kalman_init(double ave1, double ave2);
void read_SPI(void);
void process_sig(void);
void find_median(void);

void timer0ISR(void)__irq;
void timer1ISR(void)__irq;


unsigned status;       //status for PLL initialization
unsigned readcam;      //read camera flag
unsigned analogread;   //camera input ready flag
unsigned pidcompute;   //PID computation flag
unsigned setmotor;

unsigned max_index;      //maximum camera read index
unsigned filtered_index; //filtered camera index
unsigned setpoint_index; //Camera Setpoint Index - static
signed cam_error;
signed last_cam_error;
unsigned servo_setout;
unsigned motor_setout;
unsigned servo_output;
unsigned motor_output;

unsigned readcount;     //Read Out  Counter
unsigned int tickcount; //timer match 3 interrupt counter

unsigned i;             //loop index - main loop
unsigned j;             //loop index - find_median and process_sig
unsigned k;             //loop index - find_median

int min;                //process_sig variables    
int max;
int threshcount;

int config;             //initial configuration flag

char values[10];        //SPI variables
short SPI_DATAX;
short SPI_DATAY;
short SPI_DATAZ;

short aval;
short data[128];       //128 analog values
short* dataptr;

signed deriv[128];     //process_sig arrays
signed* derivptr;
double  abdata[128];
double*  abdataptr;

unsigned testdata[10]; //10 camera reads
unsigned temp_testdata;//find_median temp for sorting
int testindex;         
int exscale;

struct KALMAN
{
  double X[2][2];     //expected position (pos1, pos2; velcoity1, velocity2)
  double Ex[2][2];    //initial covariance matrix
  double A[2][2];     //state transition matrix
  //int C[2];         //measurement vector
  double Ez;          //measurement noise
  double P[2][2];     //dynamic covariance matrix
  double K[2];        //Kalman gain
  double Z[2];        //camera voltage reading offset (INPUT)
  //int I[2][2];      //identity matrix
};

struct KALMAN newFilter;
double S;            //S = C * P * C' + Ez;
double DT;           //assumed time delay
int main(void){
	
	 config = 1;
	 setmotor = 0;
	 readcount = 0;
	 tickcount = 0;
	 readcam = 0;
	 analogread = 0;
	 pidcompute = 0;
	
	 dataptr = data;
	 DT = 0.5; //.25
	 exscale = 3;
	 init_sys();	 
	 init_pwm();
	 init_timer0();
	 init_timer1();
	 init_ADC();
	 init_SPI0();	 
	 init_timer0INTR();
	 init_timer1INTR();
	
   FIO0DIR = 0x10000080;       //Servo PWM
	 FIO0MASK = 0x00000000;
	 FIO0PIN = 0x00000080;       //initialize high	 
	 
	 init_ACCSPI();	
	 motor_setout = 600;         //20% power
	 servo_setout = 86000;
	 PWMMR1 = servo_setout;      //9.75% duty cycle - STRAIGHT
	 PWMLER = 0x02;              //Enable PWM1 Latch
	 
	 //wait for a few seconds
	 for(i = 0; i < 10000000;i++){		 
	 }
	 
	 //Configure PID Setpoints & Start Motor
	 configure();
	 
   while(1){
      if(pidcompute == 1){				 
				 read_SPI();
//				 process_sig();
				
				 //find maximum of unprocessed signal
				 max_index = 20;  //first 18 readings are unpredictable
         for(i = 20; i < 110; i++){
				    if(data[i] > data[max_index]){
               max_index = i;     
            }							
				 }
				 
				 
				 
				
				 
				 
				 
				 
				 
   			 testdata[testindex] = max_index;
				 readcount++;
				 testindex++;				
				 
//					 derivptr = &deriv[0];
//					 abdataptr = &abdata[0];
					 
				 //TEST @ 10 * 25 ms = 250 ms
				 if(readcount == 10){						
						find_median();
					 
					  //KALMAN FILTER------------------------------------------------------------------------------------------------------    
						newFilter.X[0][0] = newFilter.A[0][0] * newFilter.X[0][0] + newFilter.A[0][1] * newFilter.X[1][0];
						newFilter.X[0][1] = newFilter.A[0][0] * newFilter.X[0][1] + newFilter.A[0][1] * newFilter.X[1][1];
						newFilter.X[1][0] = newFilter.X[1][0];
						newFilter.X[1][1] = newFilter.X[1][1];
						
						//P = A * P * A' + Ex;
						newFilter.P[0][0] = newFilter.A[0][0]*(newFilter.A[0][0]*newFilter.P[0][0] + newFilter.A[0][1]*newFilter.P[1][0]) +
											newFilter.A[0][1]*(newFilter.A[0][0]*newFilter.P[0][1] + newFilter.A[0][1]*newFilter.P[1][1]) +
											newFilter.Ex[0][0];
						newFilter.P[0][1] = newFilter.A[1][1]*(newFilter.A[0][0]*newFilter.P[0][1] + newFilter.A[0][1]*newFilter.P[1][1]) +
											newFilter.Ex[0][1];
						newFilter.P[1][0] = newFilter.A[0][0]*(newFilter.A[1][1]*newFilter.P[1][0]) +
											newFilter.A[0][1]*(newFilter.A[1][1]*newFilter.P[1][1]) + newFilter.Ex[1][0];
						newFilter.P[1][1] = newFilter.A[1][1]*(newFilter.A[1][1]*newFilter.P[1][1]) + newFilter.Ex[1][1];
											
						//S = C * P * C' + Ez;
						S = newFilter.P[0][0] + newFilter.Ez;
						
						//K = P * C'/S;
						newFilter.K[0] = newFilter.P[0][0]/S;
						newFilter.K[1] = newFilter.P[1][0]/S;
						
						//update sensor reading
						newFilter.Z[0] = max_index; //camera sensor
						newFilter.Z[1] = max_index; //extra sensor  
						
						//Xt = Xt + K(Z - CX);
						//adjusted X
						newFilter.X[0][0] = newFilter.X[0][0] + newFilter.K[0]*(newFilter.Z[0] - newFilter.X[0][0]);
						newFilter.X[0][1] = newFilter.X[0][1] + newFilter.K[0]*(newFilter.Z[1] - newFilter.X[0][1]);
						newFilter.X[1][0] = newFilter.X[1][0] + newFilter.K[1]*(newFilter.Z[0] - newFilter.X[0][0]);
						newFilter.X[1][1] = newFilter.X[1][1] + newFilter.K[1]*(newFilter.Z[1] - newFilter.X[0][1]);  
						
						//P = (I - (K * C))*P;
						newFilter.P[0][0] = ( (1.0 - newFilter.K[0]) * newFilter.P[0][0] );
						newFilter.P[0][1] = ( (0.0 - newFilter.K[0]) * newFilter.P[0][1] );
						newFilter.P[1][0] = ( (0.0 - newFilter.K[1]) * newFilter.P[0][0] ) + newFilter.P[1][0];
						newFilter.P[1][1] = ( (1.0 - newFilter.K[1]) * newFilter.P[0][1] ) + newFilter.P[1][1]; 
						//KALMAN FILTER--------------------------------------------------------------------------------------------------------
					  
						filtered_index = (unsigned)newFilter.X[0][0]; 
						
						cam_error = (signed)filtered_index - (signed)setpoint_index;					 
					  motor_output = 600 - (unsigned)abs((int)(cam_error*5));            
            setmotor = 1;
            
					  servo_output = servo_setout + (cam_error * 1500);
					  if(servo_output < 63000){       //min boundary
						   servo_output = 63000;
						}
						else if(servo_output > 117600){
						   servo_output = 117600;	      //max boundary
						}						
						
						PWMMR1 = servo_output;
						PWMLER = 0x02;                  //Enable PWM1 Latch
						readcount = 0;
						testindex = 0;
				 }		   
				 
				 //wait for next read for PID computation
				 pidcompute = 0;			 	  
	      
      }       		 
   }  
   
} //END MAIN	 

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

void configure(void){
   while(config == 1){
       if(pidcompute == 1){				 
				 read_SPI();			
				 //find maximum of unprocessed signal
				 max_index = 20;  //first 18 readings are unpredictable
         for(i = 20; i < 128; i++){
				    if(data[i] > data[max_index]){
               max_index = i;     
            }							
				 }
				 
   			 testdata[testindex] = max_index;
				 readcount++;
				 testindex++;					
				
				 if(readcount == 10){						
						find_median();					 
						setpoint_index = max_index;					  
					  FIO0DIR |= 0x02000000; //Motor PWM	          
	          FIO0PIN |= 0x02000000; //initialize high            					 
						config = 0;						
						readcount = 0;
						testindex = 0;
					  kalman_init((double)max_index,(double)max_index);
            					 
				 }
				 
				 //wait for next read for PID computation
				 pidcompute = 0;     
      }
   }		 
}

void init_pwm(void){
	 PWMPCR = 0x0000;
	 
   PWMPR =  0x0;
   PWMMR0 = 0x000CD140;        //14 ms signal (840000)
	 PWMMR1 = 0x00014000;        //9.75% duty cycle
   
   PWMMCR = 0x02;              //Reset counter on match	
	 PWMLER = 0x03;	             //Enable PWM 0 & 1 & 2 latch
   PWMPCR = 0x0200;            //Enable PWM1 & 2output   
	 
	 PWMTCR = 0x02;              //Counter Reset
	 PWMTCR = 0x09;              //Counter and PWM Enable
	
}

void init_timer0(void){
	
   T0MCR = 0x0601;	           //MR0 interrupt & MR3 interrupt and reset
   T0TCR = 0x01;               //enable counter
   T0MR0 = 0x00000258;         //30% duty cycle
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

void read_SPI(){
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
void kalman_init(double ave1, double ave2){
	 //NUMBERS IN THE INITIALIZATION ARE ARBITRARY
 
  newFilter.X[0][0] = ave1; //initial reading
  newFilter.X[0][1] = ave2; //initial reading
  newFilter.X[1][0] = 0.00;
  newFilter.X[1][1] = 0.00;
  
  newFilter.Ex[0][0] = (.0005*exscale); // dt^4/4 * 4
  newFilter.Ex[0][1] = (.002*exscale);  // dt^3/2 * 4
  newFilter.Ex[1][0] = (.002*exscale);  // dt^3/2 * 4
  newFilter.Ex[1][1] = (.04*exscale);   // dt^2 * 4
  
  newFilter.A[0][0] = 1.0;
  newFilter.A[0][1] = DT;
  newFilter.A[1][0] = 0.0;
  newFilter.A[1][1] = 1.0;
  
  //newFilter.C[0] = 1;
  //newFilter.C[1] = 0;
  newFilter.Ez = 100;        // standard deviaton of sensor input (std^2)
  
  newFilter.P[0][0] = newFilter.Ex[0][0]; // initially set to Ex
  newFilter.P[0][1] = newFilter.Ex[0][1];
  newFilter.P[1][0] = newFilter.Ex[1][0];
  newFilter.P[1][1] = newFilter.Ex[1][1];            
  //Initial 
  newFilter.K[0] = 0.0;
  newFilter.K[1] = 0.0;
  
  //Z is initialized as input
  newFilter.Z[0] = ave1;
  newFilter.Z[1] = ave2;
  
  //Identity Matrix
	//  newFilter.I[0][0] = 1;             
	//  newFilter.I[0][1] = 0;
	//  newFilter.I[1][0] = 0;
	//  newFilter.I[1][1] = 1; 
}
void find_median(void){
	 //sort data to find median
   for(j = 0; j < 9; j++){
		  min = j;
      for(k = j + 1; k < 10; k++){
         if(testdata[k] < testdata[min]){
				    min = k; 
				 } 
      }
      temp_testdata = testdata[j];
      testdata[j] = testdata[min];
      testdata[min] = temp_testdata;			
   }
   max_index = (testdata[5]+testdata[4])/2;	 
}

void timer0ISR(void)__irq{
   if(config == 0){	
	    FIO0PIN = FIO0PIN ^ 0x02000000; //Pin 25 - motor
	 }
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
			
		  if(setmotor == 1){
				//create boundary
				if(motor_output > 600){
					 motor_output = 600;
				}	
				else if(motor_output < 300){
					 motor_output = 300;	
				}

	       T0MR0 = motor_output;  
         setmotor = 0;				
			}
		  
	 }
//	 if(T0IR == 0x01 && setmotor == 1){
//		  //create boundary
//		  if(motor_output > 600){
//				 motor_output = 600;
//			}	
//			else if(motor_output < 300){
//				 motor_output = 300;	
//			}
//	    T0MR0 = motor_output;  
//      setmotor = 0;		 
//	 }
	 
	 T0IR = 0x09;                    //reset interrupt
   VICVectAddr = 0x00000000;   	
}

void timer1ISR(void)__irq{	 
	 readcam = 1;
	 T1IR = 0x01;                    //reset interrupt
	 VICVectAddr = 0x00000000;
}





