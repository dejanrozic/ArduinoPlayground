const int spi_ss = 48; //DAC714P A0 shift regsiter
const int dac_lch = 46; // DAC714 A1 DAC Latch
uint16_t input_0, input_1; // 16 bit input values 
uint8_t byte_0, byte_1, byte_2, byte_3; // bytes for SPI transfer

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(spi_ss, OUTPUT);
  pinMode(dac_lch, OUTPUT);
  digitalWrite(spi_ss, HIGH);
  digitalWrite(dac_lch, HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //
  static uint16_t count = 0;
  input_0 = count;
  input_1 = -count;
  count += 1;
  Serial.println(input_0);
  Serial.println(input_1);
  //
  digitalWrite(spi_ss, LOW); // A0
  byte_0 = (input_1 >> 8);
  byte_1 = (input_1 & 0xFF); 
  byte_2 = (input_0 >> 8);
  byte_3 = (input_0 & 0xFF);

  SPI.transfer(byte_0);
  SPI.transfer(byte_1);
  SPI.transfer(byte_2);
  SPI.transfer(byte_3);
  digitalWrite(spi_ss, HIGH);

  digitalWrite(dac_lch, LOW);
  digitalWrite(dac_lch, HIGH);

  delay(3);
