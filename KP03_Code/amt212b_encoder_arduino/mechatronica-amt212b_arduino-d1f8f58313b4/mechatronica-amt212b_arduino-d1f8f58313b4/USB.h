const unsigned int USB_MAX_MESSAGE_LENGTH = 20;

boolean USB_ERROR = false;
const char *usbCommands[]={
 "HELP",
 "READ",
 "RUN",
 "STOP",
 "RESET",
 "VELOCITY",
 "DISTANCE",
 "CORRECT_EXAMPLE",
 "INCORRECT_EXAMPLE"
};

const char *helpList[]={
 "Prints USB commandlist on the terminal, Note: commands are case sensitive ",
 "Type READ to read the sensor once",
 "Type RUN to start continues measument",
 "Type STOP to stop continues measument",
 "Type RESET to reset settings and encoder",
 "Type VELOCITY to receive velocity data [m/s], only updates if systems runs -> type RUN to run.",
 "Type DISTANCE to recieve the traveld distance [m]], only updates if systems runs -> type RUN to run.",
 "Print an checksum example with correct data",
 "Print an checksum example with incorrect data"
};

// number of usb commands in the list
#define numUSBCommands (sizeof(usbCommands)/sizeof(char *))
#define numHelpList (sizeof(helpList)/sizeof(char *))

void printCommands(void){
  Serial.println("");
  Serial.println("The program listens to the following commands:");
  
  for(byte i=0; i< numUSBCommands; i++){
    Serial.print(" - ");
    Serial.print(usbCommands[i]);
    if (i < 2) Serial.print("\t\t\t");
    if (i == 2) Serial.print("\t\t");
    if (i == 3) Serial.print("\t\t\t");
    if (i >= 4) Serial.print("\t");
    Serial.println(helpList[i]);
  }
  Serial.println("");
}

int searchCommand(char* USB_Command){
  
  for(byte i=0; i< numUSBCommands; i++){
    if(strcmp(USB_Command,usbCommands[i]) == 0){
      return i;
    }
  }
  return -1;
}

void processCommand(char* USB_Command){

  int ID = searchCommand(USB_Command);

  if (ID > -1){
    switch(ID){
      //HELP
      case 0: printCommands(); break;
      //[BLANK]
      case 1: AMT_requestPosition(); break;
      //RUN
      case 2: AMT_Start(); break;
      //STOP
      case 3: AMT_Stop(); break;
      //RESET
      case 4: AMT_Reset(); break;
      //VELOCITY
      case 5: Serial.println(AMT_getVelocity()); break;
      //DISTANCE
      case 6: Serial.println(AMT_getDistance()); break;
      //CORRECT EXAMPLE
      case 7: AMTchecksum_Example(CORRECT); break;
      //INCORRECT EXAMPLE
      case 8: AMTchecksum_Example(INCORRECT); break;
      default: break;
    }    
  }else{
    Serial.println("UNKOWN command received");
  }
}

void USB_Listen(void){
  char inByte = 0;
  static char USBmessage[USB_MAX_MESSAGE_LENGTH];
  static unsigned int messagePos = 0;
    
  while (Serial.available()>0) {  
    inByte = Serial.read();

    // Check for end terminator and Max message size
    if ( inByte != '\n' && (messagePos < USB_MAX_MESSAGE_LENGTH - 1) ){
      USBmessage[messagePos] = inByte;
      messagePos++;
      if(messagePos == USB_MAX_MESSAGE_LENGTH-1){
        USB_ERROR = true;
      } 
    }
    // Full message recieved....
    else{
      USBmessage[messagePos] = '\0';
      messagePos = 0;

      if (!USB_ERROR){
    
        #ifdef USB_DEBUG
          Serial.println("");
          Serial.print("Recieved USB message: ");
          Serial.println(USBmessage);
        #endif   
        processCommand(USBmessage);
      }
      else{
        Serial.println("ERROR: Exceeded USB Message length, received date failed");
      }
    }
  }
}
