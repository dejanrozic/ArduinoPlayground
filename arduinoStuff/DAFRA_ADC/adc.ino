/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
*/
#define NUM_OF_AN_VAL  16
#define TOLERANC_OF_ANALOG_SENSORS 1
volatile float toleranceofSensorsABS,
                      CH0SpodnjaMeja,
                      CH0ZgornjaMeja,
                      CH1SpodnjaMeja,
                      CH1ZgornjaMeja; //tolerance of sensors


unsigned long long previousMillis  = 0;
unsigned long long currentMillis = micros();
boolean ERROR = false;
volatile uint8_t theLowADC;
volatile uint16_t theTenBitResults, theTenBitResultsCH0, theTenBitResultsCH1;
int incomingAudio;
volatile boolean izpis = 0;
int led = 12;
const int delayChannelSelect = 25;  //ATMEGA datashet says 125uS betwen ADMUX channel select
volatile uint8_t tmp;
volatile boolean externalInteruptINT5ON = 0;


/*--------------------------------------------------------------------------------------------------
 INTIALIZING SETUP LOOP
 ---------------------------------------------------------------------------------------------------*/
void setup(){
  cli();//disable interrupts

  Serial.begin(19200); //start serial communications
  InitOI();
  InitADC();
  InitExtINT();

  sei();

  /*------------------
Calculationg of torelances
  -------------------*/
  CH0SpodnjaMeja=CH0ValueDOWN();
  CH0ZgornjaMeja=CH0ValueUP();
  CH1SpodnjaMeja=CH1ValueDOWN();
  CH1ZgornjaMeja=CH1ValueUP();
  toleranceofSensorsABS=TOLERANC_OF_ANALOG_SENSORS*1024/100;


}

/*--------------------------------------------------------------------------------------------------
 Functions for CH0 and CH1 values
 ---------------------------------------------------------------------------------------------------*/

 float CH0ValueUP(void){

   return theTenBitResultsCH0 + toleranceofSensorsABS ;
 }
 float CH0ValueDOWN(void){

   return theTenBitResultsCH0 - toleranceofSensorsABS;
 }

 float CH1ValueUP(void){

 return theTenBitResultsCH1 +  toleranceofSensorsABS;

 }
  float CH1ValueDOWN(void){

 return theTenBitResultsCH1 - toleranceofSensorsABS;

 }

/*--------------------------------------------------------------------------------------------------
 INTIALIZING INPUTS & OUTPUTS
 ---------------------------------------------------------------------------------------------------*/
void InitOI(void)
{

  //pinMode(led, OUTPUT);
  DDRB |= (1 << PB7) | (1 << PB6) | (1 << PB5);
  //PB6 PIN12 -> PIN soure pin OUTPUT HIGH
  PORTB |= (1 << PB7) | (1 << PB6);
  PORTC = 0;
  //SET OUTPUT PINS
 // pinMode(led, OUTPUT);
 //Digital input disable
 //dont work on this PIN only for comparator unit AIN...
  DDRE &=  0b11011111;
  PORTE |= 0b00100000;

  //SET INPUT PINS

  //DISABLE DIGITAL INPUT ON ANALOG PINS (help by filltering, problems is with A/D convertions and noise)
}
/*--------------------------------------------------------------------------------------------------
 INTIALIZING EXTERNAL INTERRUPT
 ---------------------------------------------------------------------------------------------------*/
void InitExtINT(void)

/* Inicializacija external interruptow
/ ISCn1 ISCn0 | Sense Signal
/  0      0   | The low level of INTn generates an interrupt request
/  0      1   | Any edge of INTn generates an interrupt request
/  1      0   | The falling edge of INTn generates an interrupt request
/  1      1   | The rising edge of INTn generates an interrupt request
*/
{
  //omogoči interrupt INT5
  //enable interrupt at rising ednge of signal
  // configure pin interrupt as input with pull up resistor
  //EIMASK set
  //enable INT5 interrupt

  /* Set INT5 to interrupt on any change */
  EIMSK |= (1 << INT5);
  //External interrupt Control register B - PIN5  | Interrupt sense control
  //reset Control register values
  EICRB = 0;
  // TRIGER EVENT
 /*-----------------------------------------
              ISCX1 	ISCX0 	Trigger Event
              0 	0 	Low Level
              0 	1 	Any Logic Change
              1 	0 	Falling Edge
              1 	1 	Rising Edge
  -------------------------------------*/
  EICRB |= ~(1 << ISC50) | ~(1 << ISC51); //sense at rising edge od INT5 -> digital pin 3 PWM - PE5
}
/*--------------------------------------------------------------------------------------------------
 INTIALIZING ADC
 ---------------------------------------------------------------------------------------------------*/


void InitADC(void)
{


  /*-----------------------------------------
      PRESCALER VALUES - Atmega datasheet
  -------------------------------------------*/
  const unsigned char PS_1 = ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  const unsigned char PS_2 = (1 << ADPS0);
  const unsigned char PS_4 = (1 << ADPS1);
  const unsigned char PS_8 = (1 << ADPS1) | (1 << ADPS0);
  const unsigned char PS_16 = (1 << ADPS2);
  const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
  const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
  const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

 //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;


  ADMUX |= (1<<REFS0) | (1<<REFS1) | (1 << MUX0); //VCC with Cap at AVCC -> External AVCC comaprator
  //ADMUX &= 0b11111001;
  //  ADMUX |= (1 << REFS0); //set reference voltage
  //  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  /*------------------------------

    ADC PS SELECT

   --------------------------------*/

  // Set up ADC
    ADCSRA &= ~PS_128; //Remove bits set by Arduino libary
    ADCSRA |=  PS_128 ;// | (1 << ADPS0); //set ADC clock with 128 prescaler- 16MHz/128= 125 kHz, 125 khz / 13 = 9615 Hz

   // Enable auto TRIGGERs
   //vect
   // ### AUTO TRIGGER
   //set ADATE to "1", ADC os set to auto triger [default is set to 0]-> single mode

   ADCSRA |= (1 << ADATE); //enabble auto trigger //without this line you muss set ADSC in ADCSRA register at end of every conversion


  //Enable Interrupt at AD convertion
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  //Enable ADC
  //Set ADC to free run mode
  // ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Delay needed for the stabilization of the ADC input voltage #Datasheet 125uS
  delayMicroseconds(125);
  ADCSRA |= (1 << ADEN); //enable ADC

  // ADCSRA |= (1 << ADSC); //start ADC measurements //does this work if interrupt is disabled?

  //sei();//enable interrupts

  ADCSRA |= (1 << ADSC); //start ADC measurements
}

/*--------------------------------------------------------------------------------------------------
 CAPTURE ISR
 ---------------------------------------------------------------------------------------------------*/

            ISR(ADC_vect) {//when new ADC value ready

                  cli();
                //  ADCSRA |= ~(1 << ADEN); //disable ADC
                  //rewrite ADMUX register
                  //AND it with 0b00000111

                  //check which channel is in use
                  //save value
                  //increment mux
                  //if chanel 1 is selected (2th. channel - start with 0), reset MUX at channel 0

                  //save MUX
                  tmp = ADMUX;
                  tmp &= 0x07;

                  //uint8_t muxRegisterValue = ADMUX;


                  theLowADC = ADCL; //  ADCL read
                  theTenBitResults = ADCH <<8 | theLowADC; //read ADCH

                  //AND it with 0b00000111 # lAST THREE VALUE IN ADMUX register are channel select bits 2^3 = 8 difffrent values CH0 - CH7

                  //Check which channel is selected and math theTenBitResults
                  //In case tmp == 0; ADMUX channel = 0; if tmp == 1 channel 1 is selected
                  //in switch case we calculate AD conversion and select another ADMUX channel
                  switch ( tmp ){

                    case 0:
                      theTenBitResultsCH0 = theTenBitResults;
                      //ADMUX = ADMUX & 0b11111001;  //Select Channel 1 n
                      //Select channel 1
                      ADMUX |= (1 << MUX0);   //Select Channel 1

                      //wait for Channel select ATMEGA says 125uS
                      delayMicroseconds(delayChannelSelect);

                       //ADCSRA |= 1<<ADSC; //start another convert || we dont need it, we have auto trigger
                      break;



                    case 1:
                      theTenBitResultsCH1 = theTenBitResults;
                      //Select channel 0
                      ADMUX &= ~(1 << MUX0);   //Select Channel 0
                      //Delay for channel select
                       delayMicroseconds(delayChannelSelect);
                       //ADCSRA |= 1<<ADSC; //start another convert || we dont need it, we have auto trigger

                      break;

                  }//end of switch case

                  /*

                  if ( tmp == 0)
                  {
                       theTenBitResultsCH0 = theTenBitResults;
                       //ADMUX = ADMUX & 0b11111001;  //Select Channel 1


                       ADMUX |= (1 << MUX0);   //Select Channel 1
                       delayMicroseconds(delayChannelSelect);

                       ADCSRA |= 1<<ADSC; //start another convert || we dont need it, we have auto trigger
                  }
                  else if ( tmp == 1)
                  {

                       theTenBitResultsCH1 = theTenBitResults;

                       ADMUX &= ~(1 << MUX0);   //Select Channel 0
                       delayMicroseconds(delayChannelSelect);
                       ADCSRA |= 1<<ADSC; //start another convert || we dont need it, we have auto trigger

                  }
                  else{}
                 */

                     //   ADCSRA |= (1 << ADEN); //enable ADC


                  //Toggle PINB7 - > LED pin on Arduino Bord PIN13
                 // PORTB ^= (1<<PB7);

                  //1. read from ADCL then from ADCH
                  //
                  // ADCL "lock" ADC value from changing
                  //
                  //2. read from ADCH value
                  //3. shift value and logic OR it.
                  //4. Save a value
                  //5. Start another conversion

                  // theLowADC = ADCL; //preberi nižja 2 bita  ADCL
                   //theTenBitResults = ADCH <<8 | theLowADC; //preberi ADCH

                  // ADCSRA |= 1<<ADSC; //start another convert || we dont need it, we have auto trigger


                  sei(); //enable global Interrupts
            }


/*--------------------------------------------------------------------------------------------------
 CAPTURE ISR EXTERNAL INTERRUPT
 ---------------------------------------------------------------------------------------------------*/
            //Interrupt Service Routine for INT5
ISR(INT5_vect)
{
  //create debouncing effect
  //deboucing effect
  externalInteruptINT5ON = 1;
        // PORTB ^=  (1 << PB5) ; //pin 11
	//PORTB ^= (1<<PB7);
}


/*--------------------------------------------------------------------------------------------------
 MAIN FUNCTION
 ---------------------------------------------------------------------------------------------------*/
void loop() {


  //TOGGLe piN PIN12
    //PORTB ^= (1<<PB6);
  /*
  if ((theTenBitResultsCH0 >= 800) || (theTenBitResultsCH0 >= 800)){
      //PORTB |= (1<<PB5);
      ERROR = true;
    }
    else{}

  //ce je napaka resetiramo izhod!
  if (ERROR){
  PORTB |= (1<<PB5);
  }
  else{
  }

  if (digitalRead(52)){
  ERROR = false;
  PORTB &= ~(1<<PB5);
  }
  else{}




  //set pin
    //PORTB |= (1<<PB6);
  //reset pin
    //PORTB &= ~(1 << PB6);
*/
 //Serial print value

  //Serial.print(tmp);
  //Serial.print("\t");

  //Serial.print("\t");
  //Serial.println(theTenBitResultsCH0SUM);

     if( (theTenBitResultsCH0 > ( CH0SpodnjaMeja ) && ( theTenBitResultsCH0 < (CH0ZgornjaMeja )))){

       if (  (theTenBitResultsCH1 > (CH1SpodnjaMeja ) ) && ( theTenBitResultsCH1 < (CH1ZgornjaMeja ))  ){

       }
       else{
       //javi napako
       }

    }
    else
    {
     //javi napako

    }


             /*
              if( (theTenBitResultsCH1 < 460 ) && (theTenBitResultsCH1 > 420)){
               currentMillis = millis();

               if (currentMillis - previousMillis  >= 100){
                //PORTB ^=  (1 << PB5) ; //pin 11

                 digitalWrite(led, !digitalRead(led));

                  Serial.print(float (theTenBitResultsCH0 * 5.012 / 1024.00));
                  Serial.print("\t");
                  Serial.print(theTenBitResultsCH1 * 5.012 / 1024.00);
                  Serial.print("\t");
                  Serial.print((float)toleranceofSensorsABS*5.012/1024);
                  Serial.print("\t");

                  Serial.print(CH0SpodnjaMeja);
                  Serial.print("\t");
                  Serial.print(CH0ZgornjaMeja);
                  Serial.print("\t");
                  Serial.print((float)CH1SpodnjaMeja);
                  Serial.print("\t");
                  Serial.print((float)CH1ZgornjaMeja);
                  Serial.println("\t");



                 previousMillis = currentMillis;
               }

               }

               */

              if (externalInteruptINT5ON){ //if external interrupt at INT5 accure
                Serial.println("INT5 external interrupt accure");
                currentMillis = micros();

                if (currentMillis - previousMillis  >= 20){ // deboucing effect ...wait 20us

                  PORTB ^=  (1 << PB5) ; //pin 11 toogle pin
                  previousMillis = currentMillis; //reset millis
                  externalInteruptINT5ON = 0;  //set flag at 0, when external interrupt accure set this flag at "1" in ISR

                }
                else{
                Serial.println("TOO FAST");
                }

              }


}
