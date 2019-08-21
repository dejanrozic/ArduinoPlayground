volatile int numSamples=0;
long t, t0;
volatile  byte NB_OF_READS= 200;
volatile byte values[1024];
void ArrayInit(byte nb);
volatile int blink1 = 0;
volatile int valuefull = 0;
volatile  int counter = 0;


void setup()
{
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(13, OUTPUT);
  
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2), blink, RISING);
  attachInterrupt(digitalPinToInterrupt(3), blink_m, RISING);
  ArrayInit(NB_OF_READS );
  
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX |= (0 & 0x07);    // set A0 analog input pin
  ADMUX |= (1 << REFS0);  // set reference voltage
  ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
 ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
 // ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements
}

ISR(ADC_vect)
{
  byte x = ADCH;  // read 8 bit value from ADC
  values[counter] = x;

 //Serial.print(numSamples);
 //Serial.print(" ");
 //Serial.println(values[numSamples]);
}
  
void loop()
{

   digitalWrite(13, blink1);   // turn the LED on (HIGH is the voltage level)
//Serial.println(counter);
                     // wait for a second
   // Serial.println(counter);
  if (valuefull == 1)
  {
      //Serial.print(values[10]); 
      
      for(int i = 0; i < 1024; i++){
         //float a =  values[i]/256 * 2.500;
          Serial.println(values[i]) ;
         // Serial.print(" | ");         
        }      
        //Serial.println("");  
            valuefull = 0;
  }
}

void ArrayInit (byte nb){
  for(int i = 0; i < nb; i++){
   values[i] = 0;
    }
  }
void blink(){

if(counter >= 360*2){
   blink1 = 1;
  }
  else{
     blink1 = 0;
    }
 
  counter = counter + 1;
  //counter = counter/3;
 // Serial.println(counter);  
  if(counter >= 1024){
     valuefull = 1;
     counter = 0;
    }
 //Serial.println("Blink...");
  }

  void blink_m(){

  blink1 =! blink1;
  //counter = counter - 1;
  if(counter <= 1024){
     counter = 0;
    }
 //Serial.println("Blink...");
  }


  
