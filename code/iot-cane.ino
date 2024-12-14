#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>
#include <BLEAdafruitService.h>
#include <LSM6DS3.h>



#define DEVICE_NAME       "IoT Cane"


#define MODE_OFF    0
#define MODE_FLASH  1
#define MODE_LIGHT  2

uint8_t previous_mode = MODE_OFF;
uint8_t current_mode = MODE_FLASH;


LSM6DS3 myIMU(I2C_MODE, 0x6A);
uint8_t taps = 0, previous_taps = 0;

#define MAX_POWER 50 // percentage, used for tap
uint8_t current_power = 100;
uint8_t current_color[3] = {255,0,0};


// Which pin on the Arduino is connected to the NeoPixels?
#define PIN       10 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 11 // Popular NeoPixel ring size
#define NEOPIXEL_VERSION_STRING "Neopixel v2.0"
#define MAXCOMPONENTS  4
uint8_t *pixelBuffer = NULL;
uint8_t width = 0;
uint8_t height = 0;
uint8_t stride;
uint8_t componentsValue;
bool is400Hz;
uint8_t components = 3;     // only 3 and 4 are valid values

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel neopixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 5000 // Time (in milliseconds) to pause between pixels



// BLE services
BLEBas  blebas;  // battery
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble




uint32_t vbat_pin = PIN_VBAT;             // A7 for feather nRF52832, A6 for nRF52840

#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

#define VBAT_DIVIDER      (0.6666F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (3.0F)        // Compensation factor for the VBAT divider


#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


float readVBAT(void) {
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

uint8_t mvToPercent(float mvolts) {
  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}

void setupAccelerometer()
{
  // Start with LSM6DS3 in disabled to save power
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.accelEnabled = 0;

  myIMU.begin();

  /* Values from the application note */

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);    // Turn on the accelerometer
                                                           // ODR_XL = 416 Hz, FS_XL = ±2 g
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x84);    // Enable interrupts and tap detection on Y-axis
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x90);  // Set tap threshold
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);    // Set Duration, Quiet and Shock time windows
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x00); // Single & double-tap enabled (SINGLE_DOUBLE_TAP = 1)
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x40);     // Double-tap interrupt driven to INT1 pin

  // Set up the sense mechanism to generate the DETECT signal to wake from system_off
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);
  attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), tapped, RISING);

  return;
}

void process_multitap(uint8_t num_taps){
  Serial.print("Multitap: ");
  Serial.println(num_taps);    
  switch (num_taps){
    case 2:
      current_color[0] = random(255);
      current_color[1] = random(255);
      current_color[2] = random(255);
      current_mode = MODE_FLASH;
      break;
    case 3:
      current_mode = MODE_LIGHT;
      break;
    case 4:
      current_color[0] = 255;
      current_color[1] = 0;
      current_color[2] = 0;
      current_power = MAX_POWER;
      current_mode = MODE_FLASH;
      break;

  }
}

/********************************************************* SETUP ************************************************/

void setup() {
  Serial.begin(9600);

  neopixel.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  pinMode (PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW); // Battery high current charging

  pinMode (VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW); // Allow to read battery

  // Get a single ADC sample and throw it away
  readVBAT();



    setupAccelerometer();



  Bluefruit.autoConnLed(false);
  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Diego Peinador Industries");
  bledis.setModel("IoT Cane beta-01");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
}


void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Advertising with only board ID
  struct ATTR_PACKED {
    uint16_t mfr_id;
    
    uint8_t  field_len;
    uint16_t field_key;
    uint16_t field_value;
  } mfr_adv;

  mfr_adv.mfr_id = UUID16_COMPANY_ID_ADAFRUIT;
  mfr_adv.field_len = 4;
  mfr_adv.field_key = 1; // board id
  mfr_adv.field_value = USB_PID;

  Bluefruit.Advertising.addManufacturerData(&mfr_adv, sizeof(mfr_adv));

  // Add name to advertising, since there is enough room
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

unsigned long currentMillis;

unsigned long bat_startMillis; 
const unsigned long bat_period = 10000;  //every 10 sec check battery

unsigned long flash_startMillis;  
const unsigned long flash_period = 50;  // 20 times per sec update color brightness

unsigned long taps_startMillis;  
const unsigned long taps_period = 1000;  // 1 sec timeout for multitap


void flash_on_tap(){
  // flash it!
  if (current_power > 0 && currentMillis - flash_startMillis >= flash_period){
    flash_startMillis = currentMillis;
    float factor = (current_power/100.0);
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      neopixel.setPixelColor(i, (int)floor(current_color[0]*factor), 
                                (int)floor(current_color[1]*factor),
                                (int)floor(current_color[2]*factor));

      neopixel.show();   // Send the updated pixel colors to the hardware.
    }
    current_power = (int)floor(current_power * 0.9);
    if (current_power == 0){
        neopixel.clear();
        neopixel.show();
    }
  }
}

void turn_on_light(){
    current_color[0] = 255;
    current_color[1] = 255;
    current_color[2] = 255;
    current_power = 70;

    float factor = (current_power/100.0);
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      neopixel.setPixelColor(i, (int)floor(current_color[0]*factor), 
                                (int)floor(current_color[1]*factor),
                                (int)floor(current_color[2]*factor));

    }
    neopixel.show();   // Send the updated pixel colors to the hardware.
}

/********************************************************* LOOP ************************************************/

void loop() {

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - bat_startMillis >= bat_period)  //test whether the period has elapsed
  {
    bat_startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.

    // Get a raw ADC reading
    float vbat_mv = readVBAT();

    // Convert from raw mv to percentage (based on LIPO chemistry)
    uint8_t vbat_per = mvToPercent(vbat_mv);

    // Display the results

    Serial.print("LIPO: ");
    Serial.print(vbat_mv);
    Serial.print(", BAT: ");
    Serial.println(vbat_per);

    blebas.write(vbat_per);
  }



  if (current_mode != previous_mode){
    previous_mode = current_mode;

    // Mode initialization
    switch (current_mode){
      case MODE_FLASH:
      break;
      case MODE_LIGHT:
        turn_on_light();
      break;
    }
  }

  // Mode animation
  switch (current_mode){
    case MODE_FLASH:
      flash_on_tap();
    break;
    case MODE_LIGHT:
    break;
  }




  // Multitap management
  if (taps > 0){
    // taps changed
    uint8_t diff_taps = taps - previous_taps;
    if (currentMillis - taps_startMillis >= taps_period){
      if (taps_startMillis == 0){ // start of taps
        taps_startMillis = currentMillis;
      } else { // end of taps
        taps_startMillis = 0;
        taps = 0;
      }
    }else if (diff_taps > 0){
      process_multitap(taps);
      previous_taps = taps;
    }
  }



  // BLE commands
  if ( Bluefruit.connected() && bleuart.notifyEnabled() )
  {
    int command = bleuart.read();

    switch (command) {
      case 'V': {   // Get Version
          commandVersion();
          break;
        }
  
      case 'S': {   // Setup dimensions, components, stride...
          commandSetup();
          break;
       }

      case 'C': {   // Clear with color
          commandClearColor();
          break;
      }

      case 'B': {   // Set Brightness
          commandSetBrightness();
          break;
      }
            
      case 'P': {   // Set Pixel
          commandSetPixel();
          break;
      }
  
      case 'I': {   // Receive new image
          commandImage();
          break;
       }

    }
  }
}


// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

void swapBuffers()
{
  uint8_t *base_addr = pixelBuffer;
  int pixelIndex = 0;
  for (int j = 0; j < height; j++)
  {
    for (int i = 0; i < width; i++) {
      if (components == 3) {
        neopixel.setPixelColor(pixelIndex, neopixel.Color(*base_addr, *(base_addr+1), *(base_addr+2)));
      }
      else {
        neopixel.setPixelColor(pixelIndex, neopixel.Color(*base_addr, *(base_addr+1), *(base_addr+2), *(base_addr+3) ));
      }
      base_addr+=components;
      pixelIndex++;
    }
    pixelIndex += stride - width;   // Move pixelIndex to the next row (take into account the stride)
  }
  neopixel.show();

}

void commandVersion() {
  Serial.println(F("Command: Version check"));
  sendResponse(NEOPIXEL_VERSION_STRING);
}

void commandSetup() {
  Serial.println(F("Command: Setup"));

  width = bleuart.read();
  height = bleuart.read();
  stride = bleuart.read();
  componentsValue = bleuart.read();
  is400Hz = bleuart.read();

  neoPixelType pixelType;
  pixelType = componentsValue + (is400Hz ? NEO_KHZ400 : NEO_KHZ800);

  components = (componentsValue == NEO_RGB || componentsValue == NEO_RBG || componentsValue == NEO_GRB || componentsValue == NEO_GBR || componentsValue == NEO_BRG || componentsValue == NEO_BGR) ? 3:4;
  
  Serial.printf("\tsize: %dx%d\n", width, height);
  Serial.printf("\tstride: %d\n", stride);
  Serial.printf("\tpixelType %d\n", pixelType);
  Serial.printf("\tcomponents: %d\n", components);

  if (pixelBuffer != NULL) {
      delete[] pixelBuffer;
  }

  uint32_t size = width*height;
  pixelBuffer = new uint8_t[size*components];
  neopixel.updateLength(size);
  neopixel.updateType(pixelType);
  neopixel.setPin(PIN);

  // Done
  sendResponse("OK");
}

void commandSetBrightness() {
  Serial.println(F("Command: SetBrightness"));

   // Read value
  uint8_t brightness = bleuart.read();

  // Set brightness
  neopixel.setBrightness(brightness);

  // Refresh pixels
  swapBuffers();

  // Done
  sendResponse("OK");
}

void commandClearColor() {
  Serial.println(F("Command: ClearColor"));

  // Read color
  uint8_t color[MAXCOMPONENTS];
  for (int j = 0; j < components;) {
    if (bleuart.available()) {
      color[j] = bleuart.read();
      j++;
    }
  }

  // Set all leds to color
  int size = width * height;
  uint8_t *base_addr = pixelBuffer;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < components; j++) {
      *base_addr = color[j];
      base_addr++;
    }
  }

  // Swap buffers
  Serial.println(F("ClearColor completed"));
  swapBuffers();


  if (components == 3) {
    Serial.printf("\tclear (%d, %d, %d)\n", color[0], color[1], color[2] );
  }
  else {
    Serial.printf("\tclear (%d, %d, %d, %d)\n", color[0], color[1], color[2], color[3] );
  }
  
  // Done
  sendResponse("OK");
}

void commandSetPixel() {
  Serial.println(F("Command: SetPixel"));

  // Read position
  uint8_t x = bleuart.read();
  uint8_t y = bleuart.read();

  // Read colors
  uint32_t pixelOffset = y*width+x;
  uint32_t pixelDataOffset = pixelOffset*components;
  uint8_t *base_addr = pixelBuffer+pixelDataOffset;
  for (int j = 0; j < components;) {
    if (bleuart.available()) {
      *base_addr = bleuart.read();
      base_addr++;
      j++;
    }
  }

  // Set colors
  uint32_t neopixelIndex = y*stride+x;
  uint8_t *pixelBufferPointer = pixelBuffer + pixelDataOffset;
  uint32_t color;
  if (components == 3) {
    color = neopixel.Color( *pixelBufferPointer, *(pixelBufferPointer+1), *(pixelBufferPointer+2) );
    Serial.printf("\tcolor (%d, %d, %d)\n",*pixelBufferPointer, *(pixelBufferPointer+1), *(pixelBufferPointer+2) );
  }
  else {
    color = neopixel.Color( *pixelBufferPointer, *(pixelBufferPointer+1), *(pixelBufferPointer+2), *(pixelBufferPointer+3) );
    Serial.printf("\tcolor (%d, %d, %d, %d)\n", *pixelBufferPointer, *(pixelBufferPointer+1), *(pixelBufferPointer+2), *(pixelBufferPointer+3) );    
  }
  neopixel.setPixelColor(neopixelIndex, color);
  neopixel.show();

  // Done
  sendResponse("OK");
}

void commandImage() {
  Serial.printf("Command: Image %dx%d, %d, %d\n", width, height, components, stride);
  
  // Receive new pixel buffer
  int size = width * height;
  uint8_t *base_addr = pixelBuffer;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < components;) {
      if (bleuart.available()) {
        *base_addr = bleuart.read();
        base_addr++;
        j++;
      }
    }

/*
    if (components == 3) {
      uint32_t index = i*components;
      Serial.printf("\tp%d (%d, %d, %d)\n", i, pixelBuffer[index], pixelBuffer[index+1], pixelBuffer[index+2] );
    }
    */
  }

  // Swap buffers
  Serial.println(F("Image received"));
  swapBuffers();

  // Done
  sendResponse("OK");
}

void sendResponse(char const *response) {
    Serial.printf("Send Response: %s\n", response);
    bleuart.write(response, strlen(response)*sizeof(char));
}

void tapped() 
{
    current_power = MAX_POWER;
    taps = taps + 1;
}
