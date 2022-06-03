#include <Arduino.h>
#include <SPI.h>
#include "printf.h"
#include <nRF24L01.h>
#include "RF24.h"
#include <Bounce2.h>

#define NRF_CE_PIN 7
#define NRF_CSN_PIN 8

// #define IS_TX_NODE // comment to set as RX node, uncomment to set as TX node

// Debouncer for button
#ifdef IS_TX_NODE
Bounce bounce = Bounce();
Bounce blink_bounce = Bounce();
#define STOP_PIN 3
#define BLINK_PIN 4
#else
#define RELAY_PIN_1 4
#define RELAY_PIN_2 5
#define RELAY_PIN_3 9
#endif
// instantiate an object for the nRF24L01 transceiver
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// an identifying device destination
// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};


struct StopStruct {
  bool STOP = false; // true is E-stop, false is normal
  bool BLINK = false;
  bool ACK = false;  // true if a stop message is acknowledged, else false
} stop;

bool blinkStatus = false;
unsigned long last_blink = millis();

bool onButtonPushed() {

  Serial.println("Button Pressed");
  
  stop.STOP = !stop.STOP;
  bool report = radio.write(&stop, sizeof(stop));

  Serial.print("Sending: ");
  Serial.println(stop.STOP);

  if (report) {
    uint8_t pipe;
    if (!radio.available(&pipe)) { // is there an ACK payload?
      Serial.println(F("Recieved: an empty ACK packet"));
    }
  } else { // transmission failed or timed out. retry send
    Serial.println(F("Transmission failed or timed out"));
  }

  return true;
}

bool onButtonFlipped() {

  Serial.println("Button Flipped");
  
  stop.BLINK = !stop.BLINK;
  bool report = radio.write(&stop, sizeof(stop));

  if (report) {
    uint8_t pipe;
    if (!radio.available(&pipe)) { // is there an ACK payload?
      Serial.println(F("Recieved: an empty ACK packet"));
    }
  } else { // transmission failed or timed out. retry send
    Serial.println(F("Transmission failed or timed out"));
  }

  return true;
}

void setup() {

  Serial.begin(115200);
  // while (!Serial) {
  //   // some boards need to wait to ensure access to serial over USB
  // }
  // radio.powerUp();
  while (!radio.begin()) {
    Serial.println(F("Radio hardware is not responding!!"));
    delay(10);// while (1) {} // hold in infinite loop
  }
  radio.setChannel(115); // the old code used this, so why not reuse it?
  radio.setPALevel(RF24_PA_MAX); // set power level
  radio.setDataRate(RF24_250KBPS);

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized
  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  
#ifdef IS_TX_NODE
  bounce.attach(STOP_PIN, INPUT_PULLUP);
  bounce.interval(500);
  blink_bounce.attach(BLINK_PIN, INPUT_PULLUP);
  blink_bounce.interval(500);
  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[0]);     // always uses pipe 0
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[1]); // using pipe 1

  radio.stopListening();
#else

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(RELAY_PIN_3, HIGH);
  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[1]);     // always uses pipe 0
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[0]); // using pipe 1
  // setup the ACK payload & load the first response into the FIFO
  // load the payload for the first received transmission on pipe 0
  radio.writeAckPayload(1, &stop, sizeof(stop));

  radio.startListening(); // put radio in RX mode
#endif
  delay(100);
  Serial.println("Setup complete");

}

void loop() {

#ifdef IS_TX_NODE
  bounce.update();
  blink_bounce.update();
  if ( bounce.changed() ) {
    // THE STATE OF THE INPUT CHANGED
    // GET THE STATE
    const bool debouncedInput = bounce.read();
    // IF THE CHANGED VALUE IS HIGH
    // if ( deboucedInput == HIGH ) {
      onButtonPushed();
    // } 
  }
  if ( blink_bounce.changed() ) {
    onButtonFlipped();
  }
#else
  uint8_t pipe;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
    StopStruct received;
    radio.read(&received, sizeof(received));

    // save incoming STOP
    stop.STOP = received.STOP;
    stop.BLINK = received.BLINK;
    stop.ACK = true;
    Serial.print("Received ");
    Serial.print(bytes);                           // print the size of the payload
    Serial.print(F(" bytes with content: STOP="));
    Serial.print(received.STOP);
    Serial.println(received.BLINK);
    radio.writeAckPayload(1, &stop, sizeof(stop));
    digitalWrite(RELAY_PIN_1, !stop.STOP); // this should allow turning on or off
    digitalWrite(RELAY_PIN_2, !stop.STOP); // this should allow turning on or off
  }
  unsigned long now = millis();
  // if (stop.BLINK && (now - last_blink > 1000)) 
  // {
  //   last_blink = now;
  //   blinkStatus = !blinkStatus;
  // } else
  // {
  //   blinkStatus = HIGH;
  // }
  // digitalWrite(RELAY_PIN_3, blinkStatus); 
  if (stop.BLINK) {
    if (now - last_blink > 1000)
    {
      last_blink = now;
      blinkStatus = !blinkStatus;
    }
  } else {
    blinkStatus = HIGH;
  }
  digitalWrite(RELAY_PIN_3, blinkStatus); 
#endif
 delay(10);
}
