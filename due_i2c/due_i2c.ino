#define GYRO_ADDRESS 0x6b
#define GYRO_WHOAMI_REG 0x0F 
#define GYRO_IDENTITY 0xD7
#define GYRO_CTRL_REG1 0x20  
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24
#define GYRO_RATE_HZ 760;

double gyro[3] = {0,0,0};   
long prevtime = micros();
long nowtime, deltatime;
double deltatimeseconds = 0.001;
boolean gyroInitialized = false;
/****************/

int i2cspeed = 10;


void setup() {
   Serial.begin(9600);
   delay(1000);
   Serial.println("booting");

   byte whoami = i2cGet(GYRO_ADDRESS, GYRO_WHOAMI_REG, 9,8);  //GETS THE ADDRESS FROM THE GYRO //ACCORDING TO THE DATASHEET 215.
   Serial.println(whoami);  //THIS IS TO TEST i2c is working..
   initializeGyro(9,8);     //Sets speed and feed and enables the XYZ
}



void loop() {
  
  delay(20);
 // getCtrl();
 // gyroStatus();
  readGyroRaw(9,8);

  Serial.print("{ \"g1x\" :\"");
  Serial.print(gyro[0]); 
  Serial.print("\", \"g1y\" :\"");
  Serial.print(gyro[1]); 
  Serial.print("\", \"g1z\" :\"");
  Serial.print(gyro[2]); 
  Serial.println("\"}");
  delay(20);
}



void initializeGyro(int sda, int scl) {
  while (gyroInitialized == false) {
    beginTransmission(GYRO_ADDRESS,sda, scl);
    i2c_write(GYRO_WHOAMI_REG,sda, scl);    
    endTransmission(sda, scl);  
    
    requestFrom(GYRO_ADDRESS,sda, scl);  
    byte i2cid = i2c_read(sda, scl);
    endTransmission(sda, scl); 
    Serial.println(i2cid);
    if (i2cid == GYRO_IDENTITY)
    {
      //success
      beginTransmission(GYRO_ADDRESS,sda, scl);
      i2c_write(GYRO_CTRL_REG1,sda, scl);   
      i2c_write(15,sda, scl);
      //i2c_write(0xFF);       //page 31/32 datasheet. 760hz //DATARATE
      
      endTransmission(sda, scl);    
      gyroInitialized = true;  
    } else { Serial.println("Gyro error cannot connect"); }
  }  
}


void readGyroRaw(int sda, int scl) {   
    uint8_t xlg,xhg, ylg, yhg, zlg, zhg;
    xlg = getReading(0x28, sda, scl);
    xhg = getReading(0x29, sda, scl);
    ylg = getReading(0x2A, sda, scl);
    yhg = getReading(0x2B, sda, scl);
    zlg = getReading(0x2C, sda, scl);
    zhg = getReading(0x2D, sda, scl);
    
    int gyroraw[3];
    gyroraw[0] = (int16_t)(xhg << 8 | xlg);
    gyroraw[1] = (int16_t)(yhg << 8 | ylg);
    gyroraw[2] = (int16_t)(zhg << 8 | zlg);
    gyro[0] = (double) gyroraw[0];
    gyro[1] = (double) gyroraw[1];
    gyro[2] = (double) gyroraw[2];
}


void getAddr(int sda, int scl) {
  beginTransmission(GYRO_ADDRESS,sda, scl);
  i2c_write(GYRO_WHOAMI_REG,sda, scl);    
  endTransmission(sda, scl);    
  requestFrom(GYRO_ADDRESS,sda, scl);  
  byte i2cid = i2c_read(sda, scl);
  Serial.println(i2cid);
  endTransmission(sda, scl);   
}

void getCtrl(int sda, int scl) {
  beginTransmission(GYRO_ADDRESS,sda, scl);
  i2c_write(0x20 ,sda, scl);    
  endTransmission(sda, scl);    
  requestFrom(GYRO_ADDRESS,sda, scl);  
  byte reply = i2c_read(sda, scl);
  endTransmission(sda, scl);   
  Serial.println(reply);  
}

void gyroStatus(int sda, int scl) {
  beginTransmission(GYRO_ADDRESS,sda, scl);
  i2c_write(0x27,sda, scl);    
  endTransmission(sda, scl);    
  requestFrom(GYRO_ADDRESS,sda, scl);  
  byte reply = i2c_read(sda, scl);
  endTransmission(sda, scl);   
  
  if (reply != 255) {
  Serial.println("status");
  Serial.println(reply);
  }  
  
}



byte i2cGet(byte devaddress, byte regaddress, int sda, int scl) {
  beginTransmission(devaddress, sda, scl);
  i2c_write(regaddress, sda, scl);  
  endTransmission(sda, scl);  
  requestFrom(devaddress, sda, scl);  
  byte data = i2c_read(sda, scl);
  endTransmission(sda, scl); 
  return data;
}


uint8_t getReading(uint8_t address, int sda, int scl) {
    beginTransmission(GYRO_ADDRESS, sda, scl);    
    i2c_write(address, sda, scl);
    endTransmission(sda, scl);
    requestFrom(GYRO_ADDRESS, sda, scl);
    endTransmission(sda, scl);    
    uint8_t reply = i2c_read(sda, scl); nack(sda, scl);
    endTransmission(sda, scl);
    return reply;
}

void ack(int sda, int scl) {
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, HIGH);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, LOW);
  digitalWrite(sda, HIGH);
}

void nack(int sda, int scl) {
  pinMode(sda, OUTPUT);  
  digitalWrite(sda, HIGH);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, HIGH);
  delayMicroseconds(i2cspeed);
  digitalWrite(scl, LOW);
  digitalWrite(sda, HIGH);
}

void i2c_start(int sda, int scl) {
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  delayMicroseconds(i2cspeed);
  pinMode(scl, OUTPUT);
  digitalWrite(scl, LOW);
}

void i2c_stop(int sda, int scl) {
  pinMode(scl, OUTPUT);
  digitalWrite(scl, HIGH);
  delayMicroseconds(i2cspeed);
  pinMode(sda, OUTPUT);
  digitalWrite(sda, HIGH);  
}


uint8_t beginTransmission(uint8_t address, int sda, int scl) {
  i2c_start(sda, scl);
  uint8_t rc = i2c_write((address<<1) | 0, sda, scl);
  return rc;
}

uint8_t beginTransmission(int address, int sda, int scl) {
  return beginTransmission( (uint8_t) address, sda, scl);
}

uint8_t requestFrom(uint8_t address, int sda, int scl)
{
    i2c_start(sda, scl);
    uint8_t rc = i2c_write((address<<1) | 1, sda, scl); // set read bit
    return rc;
}

uint8_t requestFrom(int address, int sda, int scl)
{
    return requestFrom( (uint8_t) address, sda, scl);
}

uint8_t endTransmission(int sda, int scl)
{
    i2c_stop(sda, scl);
    return 0;
}

uint8_t i2c_write( uint8_t c , int sda, int scl)
{
    for ( uint8_t i=0;i<8;i++) {
        i2c_writebit( c & 128 , sda, scl);
        c<<=1;
    }
    
    return i2c_readbit(sda, scl); //ACK
}

uint8_t i2c_read(int sda, int scl)
{
    uint8_t res = 0;

    for ( uint8_t i=0;i<8;i++) {
        res <<= 1;
        res |= i2c_readbit(sda, scl);  
        //Serial.println(res);
    }

    //i2c_writebit(0); //ACK
    return res;
}

void i2c_writebit( uint8_t c, int sda, int scl )
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
uint8_t i2c_readbit(int sda, int scl)
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