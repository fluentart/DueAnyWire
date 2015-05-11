double gyro[3] = {0,0,0};   
long prevtime = micros();
long nowtime, deltatime;
double deltatimeseconds = 0.001;
boolean gyroInitialized = false;
/****************/
int sda = 9;
int scl = 8;
int i2cspeed = 10;


void setup() {
   Serial.begin(9600);
   delay(1000);
   Serial.println("1");
   delay(1000);
   Serial.println("2");
   delay(1000);
   Serial.println("3");
   while (gyroInitialized == false) {
    Serial.println("initializing"); 
    initializeGyro();     
   }
   
   pinMode(scl, OUTPUT);
}



void loop() {
  delay(20);
 // getCtrl();
 // gyroStatus();
  readGyroRaw();

  Serial.print(gyro[0]); Serial.print(" ");
  Serial.print(gyro[1]); Serial.print(" ");
  Serial.print(gyro[2]); Serial.println(" ");
  delay(20);
}


#define GYRO_ADDRESS 0x6b
#define GYRO_WHOAMI_REG 0x0F 
#define GYRO_IDENTITY 0xD7
#define GYRO_CTRL_REG1 0x20  
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24
#define GYRO_RATE_HZ 760;


void getAddr() {
  beginTransmission(GYRO_ADDRESS);
  i2c_write(GYRO_WHOAMI_REG);    
  endTransmission();    
  requestFrom(GYRO_ADDRESS);  
  byte i2cid = i2c_read();
  Serial.println(i2cid);
  endTransmission();   
}

void getCtrl() {
  beginTransmission(GYRO_ADDRESS);
  i2c_write(0x20 );    
  endTransmission();    
  requestFrom(GYRO_ADDRESS);  
  byte reply = i2c_read();
  endTransmission();   
  Serial.println(reply);  
}

void gyroStatus() {
  beginTransmission(GYRO_ADDRESS);
  i2c_write(0x27);    
  endTransmission();    
  requestFrom(GYRO_ADDRESS);  
  byte reply = i2c_read();
  endTransmission();   
  
  if (reply != 255) {
  Serial.println("status");
  Serial.println(reply);
  }  
  
}

void initializeGyro() {
  while (gyroInitialized == false) {
    beginTransmission(GYRO_ADDRESS);
    i2c_write(GYRO_WHOAMI_REG);    
    endTransmission();  
    
    requestFrom(GYRO_ADDRESS);  
    byte i2cid = i2c_read();
    endTransmission(); 
    Serial.println(i2cid);
    if (i2cid == GYRO_IDENTITY)
    {
      //success
      beginTransmission(GYRO_ADDRESS);
      i2c_write(GYRO_CTRL_REG1);   
      i2c_write(15);
      //i2c_write(0xFF);       //page 31/32 datasheet. 760hz //DATARATE
      
      endTransmission();    
      gyroInitialized = true;  
    } else { Serial.println("Gyro error cannot connect"); }
  }  
}


void readGyroRaw() {   
    uint8_t xlg,xhg, ylg, yhg, zlg, zhg;
    xlg = getReading(0x28);
    xhg = getReading(0x29);
    ylg = getReading(0x2A);
    yhg = getReading(0x2B);
    zlg = getReading(0x2C);
    zhg = getReading(0x2D);
    
    /*
    Serial.println("readings:");
    Serial.println(xlg);
    Serial.println(xhg);
    Serial.println(ylg);
    Serial.println(yhg);
    Serial.println(zlg);
    Serial.println(zhg);    
    Serial.println();*/
    
  int gyroraw[3];
  gyroraw[0] = (int16_t)(xhg << 8 | xlg);
  gyroraw[1] = (int16_t)(yhg << 8 | ylg);
  gyroraw[2] = (int16_t)(zhg << 8 | zlg);
  gyro[0] = (double) gyroraw[0];
  gyro[1] = (double) gyroraw[1];
  gyro[2] = (double) gyroraw[2];
}

uint8_t getReading(uint8_t address) {
    beginTransmission(GYRO_ADDRESS);    
    i2c_write(address);
    endTransmission();
    requestFrom(GYRO_ADDRESS);
    endTransmission();    
    uint8_t reply = i2c_read();nack();
    endTransmission();
    return reply;
}

void ack() {
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, HIGH);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, LOW);
  digitalWrite(sda, HIGH);
}

void nack() {
  pinMode(sda, OUTPUT);  
  digitalWrite(sda, HIGH);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, HIGH);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, LOW);
  digitalWrite(sda, HIGH);
}

void i2c_start() {
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(i2cspeed);
  pinMode(scl, OUTPUT);
  digitalWrite(scl, LOW);
}

void i2c_stop() {
  pinMode(scl, OUTPUT);
  digitalWrite(scl, HIGH);
  delayMicroseconds(i2cspeed);
  pinMode(sda, OUTPUT);
  digitalWrite(sda, HIGH);  
}


uint8_t beginTransmission(uint8_t address) {
  i2c_start();
  uint8_t rc = i2c_write((address<<1) | 0);
  return rc;
}

uint8_t beginTransmission(int address) {
  return beginTransmission( (uint8_t) address);
}

uint8_t requestFrom(uint8_t address)
{
    i2c_start();
    uint8_t rc = i2c_write((address<<1) | 1); // set read bit
    return rc;
}

uint8_t requestFrom(int address)
{
    return requestFrom( (uint8_t) address);
}

uint8_t endTransmission(void)
{
    i2c_stop();
    return 0;
}

uint8_t i2c_write( uint8_t c )
{
    for ( uint8_t i=0;i<8;i++) {
        i2c_writebit( c & 128 );
        c<<=1;
    }
    
    return i2c_readbit(); //ACK
}

uint8_t i2c_read( void )
{
    uint8_t res = 0;

    for ( uint8_t i=0;i<8;i++) {
        res <<= 1;
        res |= i2c_readbit();  
        //Serial.println(res);
    }

    //i2c_writebit(0); //ACK
    return res;
}

void i2c_writebit( uint8_t c )
{
    delayMicroseconds(i2cspeed);
    
    if ( c > 0 ) {
      pinMode(sda, OUTPUT);
      digitalWrite(sda, HIGH);
    } else {
      pinMode(sda, OUTPUT);
      digitalWrite(sda, LOW);
    }
    delayMicroseconds(i2cspeed);
    
    pinMode(scl, OUTPUT);
    digitalWrite(scl, HIGH);
    delayMicroseconds(i2cspeed);

    pinMode(scl, OUTPUT);
    digitalWrite(scl, LOW);
    delayMicroseconds(i2cspeed);
   
}

//
uint8_t i2c_readbit(void)
{
    
    pinMode(sda, INPUT);
    digitalWrite(scl, LOW);
    delayMicroseconds(i2cspeed);
    digitalWrite(scl, HIGH);
      delayMicroseconds(i2cspeed);    
      uint8_t i2creadbit = digitalRead(sda);   
    digitalWrite(scl, LOW);
    
    delayMicroseconds(i2cspeed);

    return i2creadbit;
}
