#define RS485_MAX_DATA_LENGTH 4     // Maximum size(amount of bytes)of the RS485 receive Buffer
#define RS485_RW  8                 // Read / write port of the RS485 chio
/* RS485 Read/write
 *  - DE & RE Pin connected
 *  - High = Sending
 *  - Low = Reveiving
 */

byte RS485_BUFFER[RS485_MAX_DATA_LENGTH];
 
void config_RS485(void){
  
  pinMode(RS485_RW,OUTPUT);
  digitalWrite(RS485_RW,HIGH); 
}

int RS485_available(void){
  byte bytes = 0;
  boolean errorFlag = false;

  bytes = Serial1.available();                      //Check serial buffer
  if (bytes > 0) {                                  //Read buffer until empty or uptil RS485_MAX_DATA_LENGTH
    for (byte i=0; i <bytes; i++){
      if (i >= RS485_MAX_DATA_LENGTH) {
        Serial.println("WARNING: RS485 data is exceeding the buffer size!");
        errorFlag = true;
        break;
      }
      RS485_BUFFER[i] = Serial1.read();             //Store data in array of bytes
    }

    #ifdef RS485_DEBUG
      Serial.print("Incomming bytes: ");
      Serial.println(bytes);
      Serial.println("Received bytes: ");
      for (byte i=0; i <bytes; i++){
        Serial.print("Hexadecimal: 0x");
        Serial.print(RS485_BUFFER[i],HEX); 
        Serial.print(", Decimal: ");
        Serial.print(RS485_BUFFER[i],DEC);
        Serial.print(", Binair: ");
        Serial.println(RS485_BUFFER[i],BIN);
      }
      Serial.println("");  
    #endif  

    if (errorFlag) return -1;
    else return bytes;
  }
  return -1;                                                     // No new sensor data is available
}

void RS485_writeByte(byte COMMAND){
  digitalWrite(RS485_RW,HIGH);
  Serial1.write(COMMAND); 
  digitalWrite(RS485_RW,LOW);
}

void RS485_writeExtended(byte START_COMMAND,byte EXTEND_COMMAND){
  digitalWrite(RS485_RW,HIGH);
  Serial1.write(START_COMMAND); 
  Serial1.write(EXTEND_COMMAND); 
  digitalWrite(RS485_RW,LOW);
  
}
