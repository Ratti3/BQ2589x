/*                                                     https://oshwlab.com/ratti3
  _|_|_|                _|      _|      _|  _|_|_|     https://youtube.com/@Ratti3
  _|    _|    _|_|_|  _|_|_|_|_|_|_|_|            _|   https://projecthub.arduino.cc/Ratti3
  _|_|_|    _|    _|    _|      _|      _|    _|_|     https://ratti3.blogspot.com
  _|    _|  _|    _|    _|      _|      _|        _|   https://hackaday.io/Ratti3
  _|    _|    _|_|_|      _|_|    _|_|  _|  _|_|_|     https://www.hackster.io/Ratti3
                                                       https://github.com/Ratti3

This file is part of https://github.com/Ratti3/BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED
NTC code is from https://learn.adafruit.com/thermistor/using-a-thermistor

BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED is free software: you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED. If not, see <https://www.gnu.org/licenses/>.
    ____________
  -|            |-  A0 : TH_IC   [AI]         NTC for monitoring temperature close to the IC
  -|            |-  A2 : OTG     [DI]         Boost mode enable pin. The boost mode is activated when OTG_CONFIG = 1, OTG pin is high, and no input source is detected at VBUS.
  -|            |-  A3 : CE      [DI]         Active low Charge Enable pin. Battery charging is enabled when CHG_CONFIG = 1 and CE pin = Low. CE pin must be pulled High or Low.
  -|            |-  A4 : ENCA    [DI]         Rotary Encoder
  -|            |-  A5 : ENCB    [DI]         Rotary Encoder
  -| ATMEGA32U4 |-  ~5 : BUZ     [AO][PWM]    Buzzer
  -|            |-   7 : SW1     [DI][INT.6]  Rotary Encoder Switch, used to wake up from sleep mode.
  -|            |-  ~9 : INT     [DO][PCINT5] Open-drain Interrupt Output. The INT pin sends active low, 256-μs pulse to host to report charger device status and fault.
  -|            |- ~10 : PSEL    [DI]         Power source selection input. High indicates a USB host source and Low indicates an adapter source.
  -|            |- ~11 : WS2812  [DO]         WS2812B LEDs
  -|____________|- ~13 : D13_LED [DO]         LED_BUILTIN
  [DI] = Digital In, [DO] = Digital Out, [AI] = Analog In, [AO] = Analog Out
*/

// Current size of this code on the ATMEGA32U4 (SparkFun Pro Micro)
// Sketch uses 28536 bytes (99%) of program storage space. Maximum is 28672 bytes.
// Global variables use 1105 bytes (43%) of dynamic memory, leaving 1455 bytes for local variables. Maximum is 2560 bytes.

// Libraries
#include <Wire.h>
#include <EEPROM.h>            // v2.0    | https://github.com/PaulStoffregen/EEPROM
#include <BQ2589x.h>           // v1.0    | https://github.com/Ratti3/BQ2589x
#include <FastLED.h>           // v3.5.0  | https://github.com/FastLED/FastLED
#include <TimerOne.h>          // v1.1.1  | https://github.com/PaulStoffregen/TimerOne
#include <LowPower.h>          // v2.2    | https://github.com/LowPowerLab/LowPower
#include <AbleButtons.h>       // v0.3.0  | https://github.com/jsware/able-buttons
#include <BasicEncoder.h>      // v1.1.4  | https://github.com/micromouseonline/BasicEncoder
#include <Adafruit_GFX.h>      // v1.11.5 | https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>  // v2.5.7  | https://github.com/adafruit/Adafruit_SSD1306 (this needs to be modified to remove the splash screen otherwise it will not fit the ATMEGA32U4 flash)

// BQ25896
#define PIN_OTG A2         // BQ25896 OTG Digital Input
#define PIN_CE A3          // BQ25896 CE Digital Input
#define PIN_INT 9          // BQ25896 INT Digital Input
#define PIN_PSEL 10        // BQ25896 PSEL Digital Input
#define BQ2589x_ADDR 0x6B  // BQ25896 I2C Address
#define STOREVOLTAGE 3750  // VCHG Voltage for storage
bq2589x CHARGER;

// EEPROM save location, saved as byte
#define E_SAVE 0       // 0-1 - Settings saved state
#define E_VCHG 10      // arrPositionVCHG
#define E_ICHG 20      // arrPositionICHG
#define E_VOTG 30      // arrPositionVOTG
#define E_IOTG 40      // arrPositionIOTG
#define E_ROTATE 50    // oledRotation
#define E_SLEEP 60     // arraySleep
#define E_LED_BR 70    // arrPositionLED
#define E_PWR_DOWN 80  // pwrDown

// Value ranges for settings
int arrVCHG[4] = { 3840, 4096, 4192, 4208 };             // Charge voltage values (first one is used to set the STOREVOLTAGE, it is the minimum allowed value)
byte arrPositionVCHG = 2;                                // Set default charge voltage to 4.192V
int arrICHG[6] = { 512, 1024, 1536, 2048, 2560, 3072 };  // Charge current values
byte arrPositionICHG = 5;                                // Set default charge current to 3A
int arrVOTG[3] = { 4998, 5062, 5126 };                   // Boost voltage values
byte arrPositionVOTG = 1;                                // Set default boost voltage to 5.062V
int arrIOTG[5] = { 500, 750, 1200, 1650, 2150 };         // Boost current values
byte arrPositionIOTG = 4;                                // Set default boost current to 2.15A
byte oledRotation = 2;                                   // OLED default rotation setting
int arraySleep[3] = { 60, 120, 300 };                    // Sleep/Ship Mode timeout values in seconds
byte arrPositionSleep = 0;                               // Default Sleep/Ship Mode value
int arrayLED[4] = { 5, 10, 15, 20 };                     // LED brightness values
byte arrPositionLED = 0;                                 // Default LED brightness
bool pwrDOWN = 0;                                        // Enable/Disable Ship Mode

// Version number and boot text
#define TEXT1 "BQ25896"
#define TEXT2 "Battery Charger"
#define TEXT3 "Ratti3 Tech Corp"
#define VERSION "v1.0"

// AbleButtons
#define PIN_ENCODER_SW 7                 // SW1 PIN
using Button = AblePullupClickerButton;  // Button type
Button SW1(PIN_ENCODER_SW);              // The button to check is on pin

// FastLED
#define PIN_WS2812 11  // WS2812B Data PIN
#define NUM_LEDS 2     // Number of WS2812B LEDs
CRGB leds[NUM_LEDS];   // Set number of LEDs
bool stateLED0 = 0;    // Track state of LEDs

// OLED
#define OLED_ADDRESS 0x3C                   // OLED Address
Adafruit_SSD1306 OLED(128, 64, &Wire, -1);  // Adafruit OLED Class
unsigned int oledSLEEP = 0;                 // Counter for turning OLED off
unsigned long last_change = 0;              // Counter for turning OLED off
unsigned long now = 0;                      // Counter for turning OLED off

// Rotary Encoder, Timer & Menu
#define PIN_ENCA A4                                 // Rotary Encoder PIN A
#define PIN_ENCB A5                                 // Rotary Encoder PIN B
BasicEncoder ENCODER(PIN_ENCA, PIN_ENCB, HIGH, 2);  // BasicEncoder Class
int encoderPrev = 0;                                // Holds the encoder turns
bool menuMode = 0;                                  // Default menu mode
int menuPosition = 0;                               // Default menu position
bool justWokeUp = 0;                                // Track state of sleep/wake
#define MAINMENUITEMS 2                             // Number of selectable items in the main menu (count from 0)
#define SETUPMENUITEMS 8                            // Number of selectable items in the setup menu (count from 0)

// IC NTC Thermistor
#define PIN_THERMISTOR A0        // NTC Thermistor PIN
#define THERMISTORNOMINAL 10000  // NTC Thermistor nominal resistance
#define TEMPERATURENOMINAL 25    // NTC Thermistor nominal temperature
#define NUMSAMPLES 5             // Number of temperature samples to take
#define BCOEFFICIENT 3425        // NTC Thermistor B Coefficient
#define SERIESRESISTOR 10000     // NTC Thermistor series resistor
int samples[NUMSAMPLES];         // Store NTC samples

// Buzzer
#define PIN_BUZZER 5  // Buzzer PIN

void setup() {
  // Wait for things to warm up
  delay(1000);

  // I2C
  Wire.begin();

  // D13 LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // AbleButtons setup
  pinMode(PIN_ENCODER_SW, INPUT);
  SW1.begin();

  // FastLED setup
  FastLED.addLeds<WS2812, PIN_WS2812, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(arrayLED[arrPositionLED]);

  // OLED setup
  OLED.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  OLED.setTextSize(1);
  OLED.setTextColor(SSD1306_WHITE);

  // Timer1 setup
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_service);
  ENCODER.set_reverse();

  // BQ25896 setup
  // INT PIN Setup
  pinMode(PIN_INT, INPUT);
  // OTG [LOW = Off, HIGH = Boost]
  pinMode(PIN_OTG, OUTPUT);
  digitalWrite(PIN_OTG, HIGH);
  // CE [LOW = Charge, HIGH = Idle]
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);
  // PSEL [LOW = Adapter, HIGH = USB]
  pinMode(PIN_PSEL, OUTPUT);
  digitalWrite(PIN_PSEL, LOW);
  CHARGER.begin(&Wire, BQ2589x_ADDR);
  CHARGER.disable_watchdog_timer();
  CHARGER.adc_start(0);
  CHARGER.disable_charger();

  // Load (save defaults) EEPROM values
  loadEEPROM();

  // Display version on boot up
  setupDisplay();
}

void loop() {
  // Check for SW1 (Encoder Switch) presses
  SW1.handle();

  // Detect SW1 press using AbleButtons
  if (SW1.resetClicked()) {
    oledSLEEP = 0;
    OLED.dim(0);
    OLED.ssd1306_command(SSD1306_DISPLAYON);
    buttonPressed();
  }

  // Check for Encoder turns
  if (ENCODER.get_change()) {
    oledSLEEP = 0;
    OLED.dim(0);
    OLED.ssd1306_command(SSD1306_DISPLAYON);
    beepBOP(6000, 100);
    menuOption(ENCODER.get_count());
  }

  now = millis();

  unsigned int aSleep = arraySleep[arrPositionSleep];
  if (now - last_change > 1000 && oledSLEEP <= aSleep) {
    last_change = now;
    oledSLEEP++;
    if (menuMode) {
      displaySetupMenu();
    } else {
      displayStatus();
    }
    ledControl();
  } else if (oledSLEEP > aSleep && (CHARGER.is_charge_enabled() || CHARGER.is_otg_enabled())) {
    if (now - last_change > 1000) {
      last_change = now;
      ledControl();
    }
    OLED.dim(1);
    OLED.ssd1306_command(SSD1306_DISPLAYOFF);
  } else if (oledSLEEP > aSleep) {
    // If Ship Mode is enabled and timer has elasped, do a full power off
    if (pwrDOWN) {
      leds[0] = CRGB::Black;
      leds[1] = CRGB::Black;
      FastLED.show();
      CHARGER.enter_ship_mode();
    }

    // If timer for OLED display on time has expired, set the OLED contrast to 0m turn off OLED and power down the ATMEGA32U4
    OLED.dim(1);
    OLED.ssd1306_command(SSD1306_DISPLAYOFF);

    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), wakeUp, 0);

    // Put the ATMEGA32U4 to sleep
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW));

    // Wake up the OLED
    OLED.dim(0);
    OLED.ssd1306_command(SSD1306_DISPLAYON);
    // Reset the OLED sleep timer
    oledSLEEP = 0;
  }
}

// Sleep state tracking
void wakeUp() {
  justWokeUp = 1;
  beepBOP(3000, 300);
}

// Used to check encoder turns using the TimerOne service
void timer_service() {
  ENCODER.service();
}

// Default Status Display
void displayStatus() {
  OLED.clearDisplay();
  // VBUS input status
  OLED.setCursor(0, 0);
  OLED.print(" INPUT:");
  OLED.println(CHARGER.get_vbus_type_text());
  // Charge status
  OLED.setCursor(0, 10);
  OLED.print("CHARGE:");
  OLED.println(CHARGER.get_charging_status_text());
  OLED.drawLine(1, 20, 126, 20, SSD1306_WHITE);
  // VBUS Voltage
  OLED.setCursor(0, 23);
  OLED.print("VBUS:");
  OLED.println(float(CHARGER.adc_read_vbus_volt() * 0.001), 3);
  // VSYS Voltage
  OLED.setCursor(68, 23);
  OLED.print("VSYS:");
  OLED.println(float(CHARGER.adc_read_sys_volt() * 0.001), 3);
  // VBAT Voltage
  OLED.setCursor(0, 33);
  OLED.print("VBAT:");
  OLED.println(float(CHARGER.adc_read_battery_volt() * 0.001), 3);
  // Charge Current
  OLED.setCursor(68, 33);
  OLED.print("ICHG:");
  OLED.println(float(CHARGER.adc_read_charge_current() * 0.001), 3);
  // IC Temp
  OLED.setCursor(0, 43);
  OLED.print("IC");
  OLED.print(char(247));
  OLED.print("C:");
  OLED.println(ntcIC(), 1);
  // IDPM
  OLED.setCursor(68, 43);
  OLED.print("IDPM:");
  OLED.println(float(CHARGER.read_idpm_limit() * 0.001), 2);
  // Options
  OLED.drawLine(1, 53, 126, 53, SSD1306_WHITE);
  OLED.setCursor(7, 56);
  if (CHARGER.is_charge_enabled()) {
    OLED.print("STOP");
  } else {
    OLED.print("CHARGE");
  }
  OLED.setCursor(61, 56);
  if (CHARGER.is_otg_enabled()) {
    OLED.print("STOP");
  } else {
    OLED.print("OTG");
  }
  OLED.setCursor(97, 56);
  OLED.print("SETUP");

  // Set cursor position for the > selector on the main menu
  byte X = 0;
  byte Y = 0;
  switch (menuPosition) {
    case 0:
      X = 0;
      Y = 56;
      break;
    case 1:
      X = 54;
      Y = 56;
      break;
    case 2:
      X = 90;
      Y = 56;
      break;
  }
  OLED.setCursor(X, Y);
  OLED.print(">");
  OLED.display();
}

// Runs when the encoder is turned
void menuOption(int encoderCurr) {
  // Set the max number of menu items starting from 0
  int menuMax = 0;
  if (menuMode == 0) {
    menuMax = MAINMENUITEMS;
  } else {
    menuMax = SETUPMENUITEMS;
  }

  // Calculate the cursor position for the menus
  if (encoderCurr < encoderPrev) {
    --menuPosition;
    if (menuPosition < 0) menuPosition = menuMax;
  } else if (encoderCurr > encoderPrev) {
    ++menuPosition;
    if (menuPosition > menuMax) menuPosition = 0;
  }
  encoderPrev = encoderCurr;
}

// The event handler for the button.
void buttonPressed() {
  // Set error state
  bool errorBeep = 0;

  // Prevent encoder action if waking up
  if (justWokeUp) {
    justWokeUp = 0;
    return;
  }

  // Setup Menu
  if (menuMode) {
    switch (menuPosition) {
      case 0:  // Set Charge Voltage
        ++arrPositionVCHG;
        if (arrPositionVCHG > 3) arrPositionVCHG = 0;
        setChargeVoltage(arrPositionVCHG);
        break;
      case 1:  // Set Charge Current
        ++arrPositionICHG;
        if (arrPositionICHG > 5) arrPositionICHG = 0;
        setChargeCurrent(arrPositionICHG);
        break;
      case 2:  // Set Boost Voltage
        ++arrPositionVOTG;
        if (arrPositionVOTG > 2) arrPositionVOTG = 0;
        setOTGVoltage(arrPositionVOTG);
        break;
      case 3:  // Set Boost Current
        ++arrPositionIOTG;
        if (arrPositionIOTG > 4) arrPositionIOTG = 0;
        setOTGCurrent(arrPositionIOTG);
        break;
      case 4:  // Set OLED Rotation
        if (oledRotation == 2) {
          oledRotation = 0;
        } else {
          oledRotation = 2;
        }
        OLED.setRotation(oledRotation);
        break;
      case 5:  // Set Sleep Timer
        ++arrPositionSleep;
        if (arrPositionSleep > 2) arrPositionSleep = 0;
        break;
      case 6:  // Set LED brightness
        ++arrPositionLED;
        if (arrPositionLED > 3) arrPositionLED = 0;
        FastLED.setBrightness(arrayLED[arrPositionLED]);
        break;
      case 7:  // Toggle Ship Mode (full power down)
        pwrDOWN = !pwrDOWN;
        break;
      case 8:  // Exit Setup Menu and save settings to EEPROM
        saveEEPROM();
        menuMode = 0;
        menuPosition = 0;
        break;
    }
  } else {
    if (menuPosition == 0) {
      // Enable/disable charger, can only be enabled if there is and external power supply
      if (CHARGER.is_charge_enabled()) {
        CHARGER.disable_charger();
      } else if (CHARGER.get_vbus_type() > 0 && CHARGER.get_vbus_type() < 7) {
        CHARGER.enable_charger();
      } else {
        errorBeep = 1;
      }
    } else if (menuPosition == 1) {
      // Enable/Disable OTG, only can be enabled if there is a battery and there is no external power source
      if (CHARGER.is_otg_enabled()) {
        CHARGER.disable_otg();
      } else if (CHARGER.get_vbus_type() == 0) {
        CHARGER.enable_otg();
      } else {
        errorBeep = 1;
      }
    } else if (menuPosition == 2) {
      // Display the Setup Menu
      menuMode = 1;
      menuPosition = 0;
      displaySetupMenu();
    }
  }

  // Play an error beep if errorBeep == True
  if (errorBeep) {
    beepBOP(4000, 100);
    delay(100);
    beepBOP(2000, 100);
  } else {
    beepBOP(4000, 100);
  }
}

// Show the setup menu
void displaySetupMenu() {
  OLED.clearDisplay();
  OLED.setCursor(51, 0);
  OLED.println("SETUP");
  OLED.drawLine(1, 9, 126, 9, SSD1306_WHITE);
  OLED.setCursor(0, 13);
  OLED.print("CHG:  ");
  if (arrPositionVCHG == 0) {
    OLED.print("STORE   ");
  } else {
    OLED.print(float(CHARGER.get_charge_voltage() * 0.001), 3);
    OLED.print("V  ");
  }
  OLED.print(float(CHARGER.get_charge_current() * 0.001), 3);
  OLED.println("A");
  OLED.setCursor(0, 23);
  OLED.print("OTG:  ");
  OLED.print(float(CHARGER.get_otg_voltage() * 0.001), 3);
  OLED.print("V  ");
  OLED.print(float(CHARGER.get_otg_current() * 0.001), 3);
  OLED.println("A");
  OLED.setCursor(0, 33);
  OLED.print("ROT:  ");
  if (oledRotation == 0) {
    OLED.print("  0");
  } else {
    OLED.print("180");
  }
  OLED.print(char(247));
  OLED.print("  SLP:  ");
  OLED.println(arraySleep[arrPositionSleep]);
  OLED.setCursor(0, 43);
  OLED.print("LED:  ");
  byte LED = 0;
  if (arrPositionLED == 0) {
    LED = 25;
  } else if (arrPositionLED == 1) {
    LED = 50;
  } else if (arrPositionLED == 2) {
    LED = 75;
  } else {
    LED = 100;
  }
  OLED.print(LED);
  OLED.print("%  ");
  OLED.setCursor(72, 43);
  OLED.print("APO:  ");
  if (pwrDOWN) {
    OLED.println("ON");
  } else {
    OLED.println("OFF");
  }
  OLED.drawLine(1, 52, 126, 52, SSD1306_WHITE);
  OLED.setCursor(53, 57);
  OLED.print("SAVE");

  // Set the position of the > cursor
  byte X = 0;
  byte Y = 0;
  switch (menuPosition) {
    case 0:  // VCHG
      X = 29;
      Y = 13;
      break;
    case 1:  // ICHG
      X = 77;
      Y = 13;
      break;
    case 2:  // VOTG
      X = 29;
      Y = 23;
      break;
    case 3:  // IOTG
      X = 77;
      Y = 23;
      break;
    case 4:  // OLED Rotation
      X = 29;
      Y = 33;
      break;
    case 5:  // Sleep Timer
      X = 101;
      Y = 33;
      break;
    case 6:  // LED Brightness
      X = 29;
      Y = 43;
      break;
    case 7:  // Ship Mode
      X = 101;
      Y = 43;
      break;
    case 8:  // Exit to main screen
      X = 46;
      Y = 57;
      break;
  }
  OLED.setCursor(X, Y);
  OLED.print(">");
  OLED.display();
}

// Set the battery charge voltage
void setChargeVoltage(int volt) {
  CHARGER.set_charge_voltage(arrVCHG[volt]);
}

// Set the battery charge current
void setChargeCurrent(int curr) {
  CHARGER.set_charge_current(arrICHG[curr]);
}

// Set the OTG voltage
void setOTGVoltage(int volt) {
  CHARGER.set_otg_voltage(arrVOTG[volt]);
}

// Set the OTG current
void setOTGCurrent(int curr) {
  CHARGER.set_otg_current(arrIOTG[curr]);
}

// Calculate the temperature of the NTC mounted next to the BQ25896
float ntcIC() {
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(PIN_THERMISTOR);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  float steinhart;
  steinhart = average / THERMISTORNOMINAL;           // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                       // Invert
  steinhart -= 273.15;                               // convert absolute temp to C

  return steinhart;
}

// WS2812B LED control
void ledControl() {
  stateLED0 = !stateLED0;
  // Blink LEDs blue when OTG in on
  if (CHARGER.is_otg_enabled()) {
    if (stateLED0) {
      leds[0] = CRGB::Black;
    } else {
      leds[0] = CRGB::Blue;
    }
  } else { // Blink the LEDs depending on the charging status
    switch (CHARGER.get_charging_status()) {
      case 0:
        leds[0] = CRGB::Black;
        break;
      case 1:
        if (stateLED0) {
          leds[0] = CRGB::Black;
        } else {
          leds[0] = CRGB::Orange;
        }
        break;
      case 2:
        if (stateLED0) {
          leds[0] = CRGB::Black;
        } else {
          leds[0] = CRGB::Red;
        }
        break;
    }
  }

  // Call the charge detector function, will blink GREEN if STORAGE voltage has been reached
  detectCHG();

  FastLED.show();
}

// Detects charge voltage/current and disables charger if required
void detectCHG() {
  bool x = 0;
  // Determine if STORE mode is on, is charging and VBAT > 3.7V
  // Determine if no battery is connected
  // Check if charging  is finished
  if (CHARGER.is_charge_enabled() && CHARGER.adc_read_battery_volt() > STOREVOLTAGE && arrPositionVCHG == 0) {
    x = 1;
  } else if (CHARGER.is_charge_enabled() && CHARGER.adc_read_charge_current() < 100) {
    x = 1;
  } else if (CHARGER.get_charging_status() == 3) {
    x = 1;
  }

  // Disble charger based on previous calculations above
  if (x) {
    for (byte i = 0; i <= 3; i++) {
      beepBOP(2000, 100);
      delay(100);
      beepBOP(3000, 100);
      delay(100);
      beepBOP(4000, 100);
    }
    CHARGER.disable_charger();
  }
}

// Save settings to EEPROM
void updateEEPROM(byte address, byte value) {
  EEPROM.update(address, value);
  leds[1] = CRGB::HotPink;
  FastLED.show();
  delay(100);
  leds[1] = CRGB::Black;
  FastLED.show();
}

// Read saved settings from EEPROM
byte readEEPROM(byte address) {
  byte val;
  val = EEPROM.read(address);
  return val;
}

// Save defaults/load saved settings from EEPROM
void loadEEPROM() {
  if (readEEPROM(E_SAVE) == 1) {
    arrPositionVCHG = readEEPROM(E_VCHG);
    arrPositionICHG = readEEPROM(E_ICHG);
    arrPositionVOTG = readEEPROM(E_VOTG);
    arrPositionIOTG = readEEPROM(E_IOTG);
    oledRotation = readEEPROM(E_ROTATE);
    arrPositionSleep = readEEPROM(E_SLEEP);
    arrPositionLED = readEEPROM(E_LED_BR);
    pwrDOWN = readEEPROM(E_PWR_DOWN);
  } else {
    // Save default values
    saveEEPROM();
  }
  // Write values to BQ25896 register
  setChargeVoltage(arrPositionVCHG);
  setChargeCurrent(arrPositionICHG);
  setOTGVoltage(arrPositionVOTG);
  setOTGCurrent(arrPositionIOTG);
}

// Save current values to EEPROM
void saveEEPROM() {
  updateEEPROM(E_VCHG, arrPositionVCHG);
  updateEEPROM(E_ICHG, arrPositionICHG);
  updateEEPROM(E_VOTG, arrPositionVOTG);
  updateEEPROM(E_IOTG, arrPositionIOTG);
  updateEEPROM(E_ROTATE, oledRotation);
  updateEEPROM(E_SLEEP, arrPositionSleep);
  updateEEPROM(E_LED_BR, arrPositionLED);
  updateEEPROM(E_PWR_DOWN, pwrDOWN);
  updateEEPROM(E_SAVE, 1);
}

// Make some noise
void beepBOP(int frequency, int duration) {
  tone(PIN_BUZZER, frequency, duration);
}

// Display boot info
void setupDisplay() {
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Red;
  FastLED.show();

  OLED.setRotation(oledRotation);
  OLED.clearDisplay();
  OLED.setCursor(43, 1);
  OLED.println(TEXT1);
  OLED.setCursor(20, 11);
  OLED.println(TEXT2);
  OLED.setCursor(16, 34);
  OLED.println(TEXT3);
  OLED.setCursor(50, 55);
  OLED.println(VERSION);
  OLED.display();
  OLED.invertDisplay(1);

  beepBOP(3000, 200);
  beepBOP(4000, 200);

  delay(2000);

  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  FastLED.show();

  OLED.invertDisplay(0);
}