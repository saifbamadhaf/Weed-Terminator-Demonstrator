#define EXPECTED_DATA_LENGTH 2                  // Amount of bytes expected from the sensor
#define DATA_TIMEOUT        1000                // Give timeout error if no data is received after 1 second

//NOTE: Assuming default adress of 0x54!!
#define NODE_ADDRESS      0x54

//Encoder Commands
#define READ_POSITION     0x00
#define READ_TURN         0X01
#define EXTENDED_COMMAND  0x02

//Extended commands
#define SET_ZERO_POSITION 0x5E
#define RESET_ENCODER     0x75

bool RUN_FLAG = false;
bool FIRST_SAMPLE_FLAG = true;                    // The first datasample of the encoder is saved as PREV_Position in order to determine direction
bool POSITION_REQUEST_FLAG = false;               // Flag to keep track of position requests to prevent position request overflows on the RS485 bus
                                                  // Will reset if data is received on the RS485 bus

int TURNS = 0;                                    // Amount of turns bij the encoder
int PREV_POSITION = 0;                            // Previous encoder position (raw encoder position data)

float RADIAN = 0;                                 // Latest encoder position in radian
float REVOLUTIONS = 0;                            // Latest overall encoder position (Turns + current position) int Revolutions, for exampe 2.15 rev
float VELOCITY = 0;                               // Velocity [m/s] of the encoder, timers are required to generate this data
float DISTANCE = 0;                               // Distance traveld [m].
float PREV_DISTANCE = 0;                          // Previous distance to derermine velocity

const float RAD_STEP_SIZE = ((2*M_PI)/16383);     // Stepsize of the sensor for conversion to rad 
const float REV_STEP_SIZE = 1/16383;              // Stepsize for one revolution

// IMPORTANT VARIABLE, please change into correct value!
const float DIST_PER_REV = 1;                     // Distance [m] traveled after on turn by the encoder

unsigned long PREV_TIME = 0;                      // Variable to track time to determine dt.
unsigned long DATA_RECEIVE_TIMER = 0;             // Timer to check if data is received in time
                                      
enum exampleData{
  CORRECT,
  INCORRECT
};

void AMT_requestPosition(void){
  byte COMMAND = NODE_ADDRESS + READ_POSITION;
  
  #ifdef AMT_DEBUG
    if (POSITION_REQUEST_FLAG){
      Serial.println("ERROR: Still waiting on position response data, cannot send new request");
    }else{
      Serial.print("Sending RS485 command: ");
      Serial.print("0x");
      Serial.println(COMMAND,HEX);     
    }
  #endif  
  
  if (!POSITION_REQUEST_FLAG){
    RS485_writeByte(COMMAND);
    POSITION_REQUEST_FLAG = true;
    DATA_RECEIVE_TIMER = millis();
  } 
}

void AMT_resetEncoder(void){
  byte START_COMMAND = NODE_ADDRESS + EXTENDED_COMMAND;
    #ifdef AMT_DEBUG
    Serial.print("Sending RS485 command: ");
    Serial.print("0x");
    Serial.print(START_COMMAND,HEX);
    Serial.print(" 0x");
    Serial.println(RESET_ENCODER,HEX);
  #endif  
  
  RS485_writeExtended(START_COMMAND,RESET_ENCODER);
}

void AMT_setZero(void){
  byte START_COMMAND = NODE_ADDRESS + EXTENDED_COMMAND;
    #ifdef AMT_DEBUG
    Serial.print("Sending RS485 command: ");
    Serial.print("0x");
    Serial.print(START_COMMAND,HEX);
    Serial.print(" 0x");
    Serial.println(SET_ZERO_POSITION,HEX);
  #endif  
    
  RS485_writeExtended(START_COMMAND,SET_ZERO_POSITION);  
}

void AMT_Start(void){
  enableInterruptTimer();                 //Enable the interrupt timer
  RUN_FLAG = true;                        //Set the RUN flag
}

void AMT_Stop(void){
  disableInterruptTimer();                //Disable the interrupt timer
  RUN_FLAG = false;                       //clear the RUN flag                               
}

void AMT_Reset(void){                     // Reset the encoder and variables
  AMT_Stop();                             // Stop running processes 
  PREV_TIME = 0;                          // Reset time stamp
  FIRST_SAMPLE_FLAG = true;               // Reset data flag
  REVOLUTIONS = 0;                        // Reset amount of revolutions
  DISTANCE = 0;                           // Reset the traveld distance
  VELOCITY = 0;                           // Reset the velocity to 0
  AMT_resetEncoder();                     // Reset the encoder
  AMT_setZero();                          // Set encoder to zero.
}

void AMT_ResetPosition(void){             // Reset the position, for example after an index pulse
  AMT_setZero();                          // Set encoder to zero to get an accurate position.
  DISTANCE = 0;                           // Reset the traveld distance
  PREV_DISTANCE = 0;                      // Reset the previous traveld distance
  REVOLUTIONS = 0;                        // Reset the revolutions of encoder  
}

void AMT_Run(void){
  if (RUN_FLAG){
    AMT_requestPosition();
  }
}

void AMT_Velocity(unsigned long CURR_TIME){         // Calculate the velocity
  unsigned long dt = 0;
  
  if (PREV_TIME != 0){                              // With one sample no velocity can be determinded,
    dt = CURR_TIME - PREV_TIME;                     // Determine the difference in time;
    if (dt > 0){                                    // Check if dt is not zero, to prevent division by zero
      VELOCITY = ((DISTANCE - PREV_DISTANCE)/dt);   // Determine the velocity
      PREV_DISTANCE = DISTANCE;                     // Set the current distance as previous distance
    }
  }  
  PREV_TIME = CURR_TIME;                            // Store the current time as previous time
}
float AMT_getVelocity(void){
  return VELOCITY;
}

float AMT_getDistance(void){
  return DISTANCE;
}

void AMT_Process(unsigned int POSITION_){
  int STEP = 0;
  
  if(FIRST_SAMPLE_FLAG){
    PREV_POSITION = POSITION_;
    FIRST_SAMPLE_FLAG = false;
  }else{
    STEP = POSITION_ - PREV_POSITION;                     // Substract the previous position from the current position
    //Postive direction 
    if( STEP < -8200) TURNS ++;                           // If the sensor was just before the 0 point previously and now over, this results in a large negative step
                                                          // This step is then at least the half of the maximum sensor value  
    //Negative direction
    if( STEP > 8200) TURNS --;                            // If the sensor was just previously over the 0 point and now it isn't, this results in a large positive step
                                                          // This step is then at least the half of the maximum sensor value  
    RADIAN = POSITION_* RAD_STEP_SIZE;                    // Calculates the current position of the encoder in radian
    REVOLUTIONS = TURNS + (POSITION_ * REV_STEP_SIZE);    // Calculates the total amount of turns + current sensor position in revolutions
    DISTANCE = REVOLUTIONS * DIST_PER_REV;                // Calculates the traveld distance in [m]
    PREV_POSITION = POSITION_;                            // Set the current position as previous position
                                              
    #ifdef AMT_DEBUG
      Serial.println("");
      Serial.print("Sensor Revolutions: ");
      Serial.println(REVOLUTIONS);
      Serial.print("Sensor position: ");
      Serial.print(RADIAN);
      Serial.println(" Radian");
      Serial.println("");
    #endif   
  }  
}

boolean AMTchecksum(unsigned int fullResponse_, boolean EXAMPLE){
  byte oddByte = 0;                           //Variable to create the odd byte
  byte evenByte = 0;                          //Variable to create the even byte
  byte oddBits = 0;                           //Counter for the odd bits
  byte evenBits = 0;                          //Counter for the even bits
  byte n = 0;                                 //Counter for the bit position for creating odd and even bytes
  
  boolean bitCheck = false;                   // Variable to check every bit
  boolean K0 = false;                         //Even bits
  boolean K1 = false;                         //Odd bits
  boolean evenCheck = false;                  //Check Flag for evenBits
  boolean oddCheck = false;                   //Check Flag for oddBits
  boolean PRINT_INFO = EXAMPLE;               //Variable to print example data
  
  K0 = (fullResponse_ & 0x4000);              // Get the K0 bit by preforming an and operation with (0100 0000 0000 0000)
  K1 = (fullResponse_ & 0x8000);              // Get the K1 bit by preforming an and operation with (1000 0000 0000 0000)

  for(int i = 0; i<14; i++){                  // Loop through the 14 bit data (bit position 0-13)
    bitCheck = ( fullResponse_ & (1 << i));   // Check every bit for 1 or 0
    
    if ((i& 1) == 0){                         // Check if the current position is even (0,2,4,6,8,10,12)
      // Even number
      bitWrite(evenByte, n, bitCheck);        // Write the bit at position n in "evenByte" for visual data checking with datasheet
      if (bitCheck) evenBits++;               // If the bit is true, increase the counter for even bits
    }else{
      // Odd number                           // Otherwise the current position is odd (1,3,5,7,9,11,13)
      bitWrite(oddByte, n, bitCheck);         // Write the bit at position n in "oddByte" for visual data checking with datasheet
      n++;                                    // Increase the counter n (this increases after every instance of even and odd)
      if (bitCheck) oddBits++;                // If the bit is true, increase the counter for odd bits
    }
  }

  if( (((evenBits & 1) == 0 ) && K0)||(((evenBits & 1) == 1 ) && !K0)) evenCheck = true;  // Check if the even bits are correct:
                                                                                          // Checks if the evenbits are even and K0 is true
                                                                                          // Or when the evenbits are uneven and K0 is false
                                                                                          
  if( (((oddBits & 1) == 1 ) && !K1)||(((oddBits & 1) == 0 ) && K1)) oddCheck = true;     // Check if the odd bits are correct, 
                                                                                          // Checks if the oddbits are uneven and K1 is zero
                                                                                          // Or when the oddbits are even and K1 is true
  
  #ifdef AMT_DEBUG
    PRINT_INFO = true;
  #endif

  // Print info in USB if in de AMT_DEBUG is true or an example request
  if (PRINT_INFO){
    Serial.print("Odd K1: ");
    Serial.print(K1);
    Serial.print(" = ");
    Serial.print(oddByte,BIN);
    Serial.print(" oddBits = ");
    Serial.println(oddBits);
  
    Serial.print("Even K0: ");
    Serial.print(K0);
    Serial.print(" = ");
    Serial.print(evenByte,BIN);
    Serial.print(" evenBits = ");
    Serial.println(evenBits);
    Serial.println("");
    
    if (oddCheck) Serial.println("The oddBits are CORRECT");
    else Serial.println("The oddBits are INCORRECT");  
    if (evenCheck) Serial.println("The evenBits are CORRECT");
    else Serial.println("The evenBits are INCORRECT");
  }

  // if the evenCheck and oddCheck Flags are both true the received data is correct, so return true
  // of one of the two flags is false, the data is incorrect and return false
  if (evenCheck && oddCheck)return true;
  else return false;
}

void AMT_Listen(void){
  int bytes = 0;
  unsigned int fullResponse = 0;
  unsigned int checksum = 0;
  unsigned int position_ = 0;

  bytes = RS485_available();
  if (bytes > 0){
    if (bytes == EXPECTED_DATA_LENGTH){
      fullResponse = ((RS485_BUFFER[1]<<8) + RS485_BUFFER[0]);
      
      #ifdef AMT_DEBUG
        Serial.println("");
        Serial.print("Full response: 0x");
        Serial.println(fullResponse,HEX);
      #endif
      
      if (AMTchecksum(fullResponse,false)){
        checksum = (fullResponse & 0xC000);   // Get the first two bits of the dataset, these are K1 and K0
        position_ = (fullResponse - checksum);  // Substract the checksum bits from the fullResponse data to get the position data

        AMT_Process(position_);
        
        #ifdef AMT_DEBUG
          Serial.println("");
          Serial.println("DATA VALIDITY VERIFIED");
          Serial.print("Position: 0x");
          Serial.println(position_,HEX);
        #endif
      }
    }else{
      Serial.print("ERROR: The received amount of: [");
      Serial.print(bytes); 
      Serial.print("] RS485 bytes does not match the exptected amount of: [");
      Serial.print(EXPECTED_DATA_LENGTH);
      Serial.println("]");
    }
    POSITION_REQUEST_FLAG = false;
  }

  if ( ((millis()-DATA_RECEIVE_TIMER)>DATA_TIMEOUT) && POSITION_REQUEST_FLAG){    //Check is data is requested and if timer is timed-out
    Serial.print("ERROR: No sensor data received within: ");                      //Print error statement
    Serial.print(DATA_TIMEOUT/1000);
    Serial.println(" second(s)");
    POSITION_REQUEST_FLAG = false;                                                //Reset flag
  }
}

void AMTchecksum_Example(exampleData dataState){
  byte MSB = 0x61;
  byte LSB = 0xAB;
  byte checksum = (MSB & 0xC0);
  unsigned int fullResponse = 0;
  unsigned int postionData = 0;

  Serial.println("");

  if (dataState == INCORRECT) {
    LSB = 0xAC;
    Serial.println("Incorrect data checksum example");
  }else{
    Serial.println("Correct data checksum example from datasheet");
  }
  
  fullResponse = ( (MSB<<8) + LSB );
  postionData = ( ((MSB-checksum)<<8)+ LSB );
  
  Serial.print("Byte 1: 0x");
  Serial.println(MSB,HEX);
  Serial.print("Byte 2: 0x");
  Serial.println(LSB,HEX);
  Serial.print("Full response: 0x");
  Serial.println(fullResponse,HEX);
  Serial.print("Checksum: ");
  Serial.println(checksum,BIN);
  Serial.print("Position Data: 0x");
  Serial.println(postionData,HEX);
  Serial.println("");

  AMTchecksum(fullResponse,true);
}
