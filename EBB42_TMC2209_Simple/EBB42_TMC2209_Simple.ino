/**
 * Author Teemu Mäntykallio
 * Modified by Alextrical, Run the motor at a set speed
 */

#include <TMCStepper.h>

HardwareSerial Serial2(PA15, PA14_ALT1);

#define EN_PIN           PD2 // Enable
#define DIR_PIN          PD1 // Direction
#define STEP_PIN         PD0 // Step
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define LED_BUILTIN      PA13

#define R_SENSE 0.11f // Match to your driver
                      // BTT EBB42/36 use 0.11

#define microSteps 64

// Select your stepper driver type
TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);       // Hardware Serial
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);     // Software serial

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  //Enable driver in hardware
  digitalWrite(DIR_PIN, LOW); //LOW or HIGH
  digitalWrite(STEP_PIN, LOW);

                                  // Enable one according to your setup
  Serial2.begin(115200);          // HW UART drivers
// driver.beginSerial(115200);     // SW UART drivers
  
  USART2->CR1 &= ~USART_CR1_UE;   // UE = 0... disable USART
  USART2->CR2 |= USART_CR2_SWAP;  //Swap TX/RX pins2
  USART2->CR3 |= USART_CR3_HDSEL; //Set Half-duplex selection
  USART2->CR1 |= USART_CR1_UE;    // UE = 1... Enable USART

	Serial.begin(9600);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }

  Serial.print(F("\nTesting connection..."));
  uint8_t result = driver.test_connection();
  if (result) {
      Serial.println(F("failed!"));
      Serial.print(F("Likely cause: "));
      switch(result) {
          case 1: Serial.println(F("loost connection")); break;
          case 2: Serial.println(F("Likely cause: no power")); digitalWrite(LED_BUILTIN, HIGH); break;
      }
      Serial.println(F("Fix the problem and reset board."));
      abort();
  }
  Serial.println(F("OK"));



  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(400);        // Set motor RMS current
  driver.microsteps(microSteps);         // Set microsteps

  // driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208//2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop

  driver.VACTUAL((microSteps*400*1)/(0.715));
}

bool shaft = false;

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);   
  // shaft = !shaft;
  // driver.shaft(shaft);
  // driver.VACTUAL((microSteps*400*1)/(0.715));
  // delay(1000);
  // driver.VACTUAL(0);
  // delay(1000);
  // shaft = !shaft;
  // driver.shaft(shaft);
  // driver.VACTUAL((microSteps*400*1)/(0.715));
  // delay(1000);
  // driver.VACTUAL(0);
  // delay(1000);
  // for (uint32_t i = (microSteps*400*4); i>0; i--) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(10);
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(10);
  // }
  // delay(1000);  
}
