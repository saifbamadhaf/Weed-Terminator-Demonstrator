#include "settings.h"
#include "RS485.h"
#include "AMT212B.h"
#include "USB.h"

#define RS485_BAUDRATE  2000000
#define RS485_CONFIG    SERIAL_8N1                // SERIAL_8N1 is the default serial setting, 8 bits / no parity / 1 stop bit

void setup() {
  Serial.begin(115200);                           // USB Serial
  Serial1.begin(RS485_BAUDRATE,RS485_CONFIG);     // RS485 Serial bus

  configISR();
  config_RS485();                                 // Configure RS485 RW pin
  AMT_setZero();                                      // Sends command to set the Encoder to zero
  Serial.println("RS485 Master started - Type HELP to receive more info");
}

void loop() {
  USB_Listen();                                   // Listen for Data on USB bus.
  AMT_Listen();                                   // Listen for new sensor data
  
  if (ISR_TIMER_FLAG && RUN_FLAG) {               // If timer interrupt and continuous measurements are running, calculate velocity
    AMT_Velocity(millis());                       // Calculate the velocity using the current timestamp
    ISR_TIMER_FLAG = false;                       // Reset the ISR timer flag
  }

  AMT_Run();                                      // If continues sampling is running this function checks a flag and requests sensor data
}
