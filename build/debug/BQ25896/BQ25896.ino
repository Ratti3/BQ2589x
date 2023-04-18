/*                                                     https://oshwlab.com/ratti3
  _|_|_|                _|      _|      _|  _|_|_|     https://youtube.com/@Ratti3
  _|    _|    _|_|_|  _|_|_|_|_|_|_|_|            _|   https://projecthub.arduino.cc/Ratti3
  _|_|_|    _|    _|    _|      _|      _|    _|_|     https://ratti3.blogspot.com
  _|    _|  _|    _|    _|      _|      _|        _|   https://hackaday.io/Ratti3
  _|    _|    _|_|_|      _|_|    _|_|  _|  _|_|_|     https://www.hackster.io/Ratti3
.                                                      https://github.com/Ratti3

This file is part of Foobar.

Foobar is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as 
published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Foobar is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty 
of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include <EEPROM.h>            // v2.0    | https://github.com/PaulStoffregen/EEPROM
#include <BQ2589x.h>           // v1.0    | https://github.com/Ratti3/BQ2589x
#include <FastLED.h>           // v3.5.0  | https://github.com/FastLED/FastLED
#include <TimerOne.h>          // v1.1.1  | https://github.com/PaulStoffregen/TimerOne
#include <LowPower.h>          // v2.2    | https://github.com/LowPowerLab/LowPower
#include <AbleButtons.h>       // v0.3.0  | https://github.com/jsware/able-buttons
#include <BasicEncoder.h>      // v1.1.4  | https://github.com/micromouseonline/BasicEncoder
#include <Adafruit_GFX.h>      // v1.11.5 | https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>  // v2.5.7  | https://github.com/adafruit/Adafruit_SSD1306

/*  ____________
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

// EEPROM
#define E_SAVE 0
#define E_VCHG 10
#define E_ICHG 20
#define E_VOTG 30
#define E_IOTG 40
#define E_ROTATE 50
#define E_SLEEP 60
#define E_LED_BR 70
#define E_OLED_BR 80
bool settingsSaved = 0;

// BQ25896
#define PIN_OTG A2         // BQ25896 OTG Digital Input
#define PIN_CE A3          // BQ25896 CE Digital Input
#define PIN_INT 9          // BQ25896 INT Digital Input
#define PIN_PSEL 10        // BQ25896 PSEL Digital Input
#define BQ2589x_ADDR 0x6B  // BQ25896 I2C Address
bq2589x CHARGER;
int arrVCHG[5] = { 3840, 4000, 4096, 4192, 4208 };       // Charge volage values
byte arrPositionVCHG = 3;                                // Set default charge voltage to 4.92V
int arrICHG[6] = { 512, 1024, 1536, 2048, 2560, 3072 };  // Charge current values
byte arrPositionICHG = 5;                                // Set default charge current to 3A
int arrVOTG[3] = { 4998, 5062, 5126 };                   // Boost voltage values
byte arrPositionVOTG = 1;                                // Set default boost voltage to 5.062V
int arrIOTG[5] = { 500, 750, 1200, 1650, 2150 };         // Boost current values
byte arrPositionIOTG = 4;                                // Set default boost current to 2.15A
byte verHigh = 1;                                        // Version number major
byte verLow = 0;                                         // Version number minor

// AbleButtons
#define PIN_ENCODER_SW 7                 // SW1 PIN
using Button = AblePullupClickerButton;  // Button type
Button SW1(PIN_ENCODER_SW);              // The button to check is on pin

// FastLED
#define PIN_WS2812 11  // WS2812B Data PIN
#define NUM_LEDS 2     // Number of WS2812B LEDs
#define LED_BRIGHTNESS 5
CRGB leds[NUM_LEDS];
bool stateLED0 = 0;

// OLED
#define OLED_ADDRESS 0x3C  // OLED Address
Adafruit_SSD1306 OLED(128, 64, &Wire, -1);
unsigned long oledSLEEP = 0;
byte oledRotation = 2;

// Rotary Encoder, Timer & Menu
#define PIN_ENCA A4  // Rotary Encoder PIN A
#define PIN_ENCB A5  // Rotary Encoder PIN B
BasicEncoder ENCODER(PIN_ENCA, PIN_ENCB, HIGH, 2);
unsigned long last_change = 0;
unsigned long now = 0;
int encoderPrev = 0;
bool menuMode = 0;
int menuPosition = 0;
bool justWokeUp = 0;
int arraySleep[3] = { 60, 120, 300 };
byte settingsSleep = 1;

// IC NTC Thermistor
#define PIN_THERMISTOR A0  // NTC Thermistor PIN
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5
#define BCOEFFICIENT 3425
#define SERIESRESISTOR 10000
int samples[NUMSAMPLES];

// Buzzer
#define PIN_BUZZER 5  // Buzzer PIN

void setup() {

  // I2C
  Wire.begin();

  // Serial
  // Serial.begin(9600);
  // Wait for Serial
  // delay(2000);

  // D13 LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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

  // AbleButtons setup
  pinMode(PIN_ENCODER_SW, INPUT);
  SW1.begin();

  // FastLED setup
  FastLED.addLeds<WS2812, PIN_WS2812, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  // OLED setup
  OLED.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  OLED.setTextSize(1);

  // Timer1 setup
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_service);
  ENCODER.set_reverse();

  // BQ25896 setup
  CHARGER.begin(&Wire, BQ2589x_ADDR);
  CHARGER.disable_watchdog_timer();
  CHARGER.adc_start(0);
  CHARGER.disable_charger();
  setChargeVoltage(3);
  setChargeCurrent(5);
  setOTGVoltage(1);
  setOTGCurrent(4);

  // Display version on boot up
  setupDisplay();
}

void loop() {
  // Check for SW1 (Encoder Switch) presses
  SW1.handle();

  if (SW1.resetClicked()) {  // Reset the click, indicating if it had been clicked.
    buttonPressed();
  }

  // Check for Encoder turns
  if (ENCODER.get_change()) {
    OLED.dim(0);
    OLED.ssd1306_command(SSD1306_DISPLAYON);
    oledSLEEP = 0;
    tone(PIN_BUZZER, 6000, 100);
    menuOption(ENCODER.get_count());
  }

  now = millis();

  unsigned int aSleep = arraySleep[settingsSleep];
  if (now - last_change > 1000 && oledSLEEP <= aSleep) {
    last_change = now;
    oledSLEEP++;
    if (menuMode) {
      displaySetupMenu();
    } else {
      displayStatus();
    }
    showLEDs();
  } else if (oledSLEEP > aSleep && (CHARGER.is_charge_enabled() || CHARGER.is_otg_enabled())) {
    if (now - last_change > 1000) {
      last_change = now;
      showLEDs();
    }
    OLED.dim(1);
    OLED.ssd1306_command(SSD1306_DISPLAYOFF);
  } else if (oledSLEEP > aSleep) {
    OLED.dim(1);
    OLED.ssd1306_command(SSD1306_DISPLAYOFF);

    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), wakeUp, 0);

    // Put the ATMEGA32U4 to sleep
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW));

    OLED.dim(0);
    OLED.ssd1306_command(SSD1306_DISPLAYON);
    oledSLEEP = 0;
  }
}

void wakeUp() {
  justWokeUp = 1;
  tone(PIN_BUZZER, 3000, 300);
}

void timer_service() {
  ENCODER.service();
}

// Default Status Display
void displayStatus() {
  OLED.clearDisplay();
  OLED.setTextColor(SSD1306_WHITE);
  // VBUS input status
  OLED.setCursor(0, 0);
  OLED.print("INPUT: ");
  OLED.println(CHARGER.get_vbus_type_text());
  // Charge status
  OLED.setCursor(0, 10);
  OLED.print("CHARGE:");
  OLED.println(CHARGER.get_charging_status_text());
  // Line
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
  int menuMax = 0;
  if (menuMode == 0) {
    menuMax = 2;
  } else {
    menuMax = 6;
  }

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
  bool errorBeep = 0;

  if (justWokeUp) {
    justWokeUp = 0;
    return;
  }

  if (menuMode) {
    switch (menuPosition) {
      case 0:
        // Set Charge Voltage
        ++arrPositionVCHG;
        if (arrPositionVCHG > 4) arrPositionVCHG = 0;
        setChargeVoltage(arrPositionVCHG);
        break;
      case 1:
        // Set Charge Current
        ++arrPositionICHG;
        if (arrPositionICHG > 5) arrPositionICHG = 0;
        setChargeCurrent(arrPositionICHG);
        break;
      case 2:
        // Set Boost Voltage
        ++arrPositionVOTG;
        if (arrPositionVOTG > 2) arrPositionVOTG = 0;
        setOTGVoltage(arrPositionVOTG);
        break;
      case 3:
        // Set Boost Current
        ++arrPositionIOTG;
        if (arrPositionIOTG > 4) arrPositionIOTG = 0;
        setOTGCurrent(arrPositionIOTG);
        break;
      case 4:
        if (oledRotation == 2) {
          oledRotation = 0;
        } else {
          oledRotation = 2;
        }
        OLED.setRotation(oledRotation);
        break;
      case 5:
        ++settingsSleep;
        if (settingsSleep > 2) settingsSleep = 0;
        ////arraySleep[settingsSleep];
        break;
      case 6:
        // Exit
        saveEEPROM(10, arrPositionVCHG);
        saveEEPROM(20, arrPositionICHG);
        saveEEPROM(30, arrPositionVOTG);
        saveEEPROM(40, arrPositionIOTG);
        saveEEPROM(50, oledRotation);
        saveEEPROM(60, settingsSleep);
        menuMode = 0;
        menuPosition = 0;
        displayStatus();
        break;
    }
  } else {
    if (menuPosition == 0) {
      // Start
      if (CHARGER.is_charge_enabled()) {
        CHARGER.disable_charger();
      } else if (CHARGER.get_vbus_type() > 0 && CHARGER.get_vbus_type() < 7) {
        CHARGER.enable_charger();
      } else {
        errorBeep = 1;
      }
    } else if (menuPosition == 1) {
      // OTG
      if (CHARGER.is_otg_enabled()) {
        CHARGER.disable_otg();
      } else if (CHARGER.get_vbus_type() == 0) {
        CHARGER.enable_otg();
      } else {
        errorBeep = 1;
      }
    } else if (menuPosition == 2) {
      // Menu
      menuMode = 1;
      menuPosition = 0;
      displaySetupMenu();
    }
  }

  if (errorBeep) {
    tone(PIN_BUZZER, 4000, 100);
    delay(100);
    tone(PIN_BUZZER, 2000, 100);
  } else {
    tone(PIN_BUZZER, 4000, 100);
  }
}

void displaySetupMenu() {
  OLED.clearDisplay();
  OLED.setCursor(34, 0);
  OLED.println("SETUP MENU");
  OLED.drawLine(1, 9, 126, 9, SSD1306_WHITE);
  OLED.setCursor(0, 12);
  OLED.print("CHG:  ");
  OLED.print(float(CHARGER.get_charge_voltage() * 0.001), 3);
  OLED.print("V  ");
  OLED.print(float(CHARGER.get_charge_current() * 0.001), 3);
  OLED.println("A");
  OLED.setCursor(0, 22);
  OLED.print("OTG:  ");
  OLED.print(float(CHARGER.get_otg_voltage() * 0.001), 3);
  OLED.print("V  ");
  OLED.print(float(CHARGER.get_otg_current() * 0.001), 3);
  OLED.println("A");
  OLED.setCursor(7, 32);
  OLED.print("Rotate Display: ");
  if (oledRotation == 0) {
    OLED.print("0");
  } else {
    OLED.print("180");
  }
  OLED.println(char(247));
  OLED.setCursor(7, 42);
  OLED.print("Sleep Timer: ");
  OLED.print(arraySleep[settingsSleep]);
  OLED.println("s");
  // Options
  OLED.setCursor(53, 57);
  OLED.print("EXIT");

  byte X = 0;
  byte Y = 0;
  switch (menuPosition) {
    case 0:
      X = 29;
      Y = 12;
      break;
    case 1:
      X = 77;
      Y = 12;
      break;
    case 2:
      X = 29;
      Y = 22;
      break;
    case 3:
      X = 77;
      Y = 22;
      break;
    case 4:
      X = 0;
      Y = 32;
      break;
    case 5:
      X = 0;
      Y = 42;
      break;
    case 6:
      X = 46;
      Y = 57;
      break;
  }
  OLED.setCursor(X, Y);
  OLED.print(">");
  OLED.display();
}

void setChargeVoltage(int volt) {
  CHARGER.set_charge_voltage(arrVCHG[volt]);
}

void setChargeCurrent(int curr) {
  CHARGER.set_charge_current(arrICHG[curr]);
}

void setOTGVoltage(int volt) {
  CHARGER.set_otg_voltage(arrVOTG[volt]);
}

void setOTGCurrent(int curr) {
  CHARGER.set_otg_current(arrIOTG[curr]);
}

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

void showLEDs() {
  stateLED0 = !stateLED0;
  if (CHARGER.is_otg_enabled()) {
    if (stateLED0) {
      leds[1] = CRGB::Black;
    } else {
      leds[1] = CRGB::Blue;
    }
  } else {
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
      case 3:
        if (stateLED0) {
          leds[0] = CRGB::Black;
        } else {
          leds[0] = CRGB::Green;
        }
        break;
    }
  }
  FastLED.show();
}

void saveEEPROM(byte address, byte value) {
  EEPROM.update(address, value);
  leds[1] = CRGB::Yellow;
  FastLED.show();
  delay(250);
  leds[1] = CRGB::Black;
  FastLED.show();
}

byte readEEPROM(byte address) {
  byte val;
  val = EEPROM.read(address);
  return val;
}

void setupDisplay() {
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Red;
  FastLED.show();

  OLED.setRotation(oledRotation);
  OLED.clearDisplay();
  OLED.setTextColor(SSD1306_WHITE);
  OLED.setCursor(43, 1);
  OLED.println("BQ25896");
  OLED.setCursor(20, 11);
  OLED.println("Battery Charger");
  OLED.setCursor(16, 34);
  OLED.println("Ratti3 Tech Corp");
  OLED.setCursor(50, 55);
  OLED.print("v");
  OLED.print(verHigh);
  OLED.print(".");
  OLED.println(verLow);
  OLED.display();
  OLED.invertDisplay(1);

  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  FastLED.show();

  tone(PIN_BUZZER, 3000, 200);
  delay(500);
  tone(PIN_BUZZER, 4000, 200);

  delay(2000);

  OLED.invertDisplay(0);
  OLED.clearDisplay();
}