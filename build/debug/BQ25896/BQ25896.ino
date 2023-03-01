#include <Wire.h>
#include <BQ2589x.h>
#include <FastLED.h>
#include <TimerOne.h>
#include <LowPower.h>
#include <AceButton.h>
#include <BasicEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*  ____________
  -|            |-  A0 : TH_IC
  -|            |-  A2 : OTG
  -|            |-  A3 : CE
  -|            |-  A4 : ENCA
  -|            |-  A5 : ENCB
  -| ATMEGA32U4 |-  ~5 : BUZ
  -|            |-   7 : SW1 [INT.6]
  -|            |-  ~9 : INT [PCINT5]
  -|            |- ~10 : PSEL
  -|            |- ~11 : WS2812
  -|____________|- ~13 : D13_LED
*/

// BQ25896
#define PIN_OTG A2
#define PIN_CE A3
#define PIN_INT 9
#define PIN_PSEL 10
#define BQ2589x_ADDR 0x6B
bq2589x CHARGER;

// AceButton
#define LONGPRESSDURATION 5000
using namespace ace_button;
#define PIN_ENCODER_SW 7
AceButton button(PIN_ENCODER_SW);
void handleEvent(AceButton*, uint8_t, uint8_t);

// FastLED
#define PIN_WS2812 11
#define NUM_LEDS 2
#define LED_BRIGHTNESS 10
CRGB leds[NUM_LEDS];
bool stateLED0 = 0;

// OLED
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 OLED(128, 64, &Wire, -1);
unsigned long oled_sleep = 0;

// Rotary Encoder & Timer
#define PIN_ENCA A4
#define PIN_ENCB A5
BasicEncoder ENCODER(PIN_ENCA, PIN_ENCB);
unsigned long last_change = 0;
unsigned long now = 0;

// IC NTC Thermistor
#define PIN_THERMISTOR A0
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define NUMSAMPLES 5
#define BCOEFFICIENT 3425
#define SERIESRESISTOR 10000
int samples[NUMSAMPLES];

// Buzzer
#define PIN_BUZZER 5

void setup() {

  Wire.begin();
  Serial.begin(9600);

  // Wait for Serial
  delay(2000);

  // D13 LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // OTG
  pinMode(PIN_OTG, OUTPUT);
  digitalWrite(PIN_OTG, LOW);

  // CE
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, HIGH);

  // PSEL
  pinMode(PIN_PSEL, OUTPUT);
  digitalWrite(PIN_PSEL, HIGH);

  pinMode(PIN_ENCODER_SW, INPUT);
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(LONGPRESSDURATION);

  FastLED.addLeds<WS2812, PIN_WS2812>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  delay(250);
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  OLED.clearDisplay();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_service);
  ENCODER.set_reverse();

  CHARGER.begin(&Wire, BQ2589x_ADDR);

  leds[0] = CRGB::Red;
  tone(PIN_BUZZER, 2000, 500);
  FastLED.show();
  delay(500);
  leds[0] = CRGB::Black;
  FastLED.show();
  leds[1] = CRGB::Green;
  tone(PIN_BUZZER, 4000, 500);
  FastLED.show();
  delay(500);
  leds[1] = CRGB::Black;
  FastLED.show();

  //Serial.println("Dis WDT");
  CHARGER.disable_watchdog_timer();
  //Serial.println("ADC Start");
  CHARGER.adc_start(0);
  //Serial.println("CRG V");
  CHARGER.set_chargevoltage(4192);
  //Serial.println("EN OTG");
  CHARGER.enable_otg();
  //Serial.println("OTG V");
  CHARGER.set_otg_volt(4998);
  //Serial.println("OTG I");
  CHARGER.set_otg_current(1400);
  /*
  // Read all REG values
  for (byte i = 0; i <=20; i++) {
    Serial.print(i, HEX); Serial.print(": "); Serial.println(CHARGER.read_reg(i), BIN);
    delay(10);
  }

  Serial.print(0x0A, HEX);
  Serial.print(": ");
  Serial.println(CHARGER.read_reg(0x0A), BIN);
  */
}

void loop() {

  button.check();

  int encoder_change = ENCODER.get_change();
  if (encoder_change) {
    Serial.println(ENCODER.get_count());
    OLED.ssd1306_command(SSD1306_DISPLAYON);
    oled_sleep = 0;
  }

  now = millis();

  if (now - last_change > 1000 && oled_sleep <= 60) {
    last_change = now;
    oled_sleep++;

    chargeLED();

    OLED.clearDisplay();
    OLED.setTextSize(1);
    OLED.setTextColor(SSD1306_WHITE);
    // VBUS input status
    OLED.setCursor(0, 0);
    OLED.print("Input: ");
    OLED.println(CHARGER.get_vbus_type_text());
    // Charge status
    OLED.setCursor(0, 10);
    OLED.print("Charge:");
    OLED.println(CHARGER.get_charging_status_text());
    // Line
    OLED.drawLine(1, 19, 126, 19, SSD1306_WHITE);
    // VBUS Voltage
    OLED.setCursor(0, 21);
    OLED.print("VBUS:");
    OLED.println(float(CHARGER.adc_read_vbus_volt() * 0.001), 3);
    // VSYS Voltage
    OLED.setCursor(68, 21);
    OLED.print("VSYS:");
    OLED.println(float(CHARGER.adc_read_sys_volt() * 0.001), 3);
    // VBAT Voltage
    OLED.setCursor(0, 31);
    OLED.print("VBAT:");
    OLED.println(float(CHARGER.adc_read_battery_volt() * 0.001), 3);
    // Charge Current
    OLED.setCursor(68, 31);
    OLED.print("ICHG:");
    OLED.println(float(CHARGER.adc_read_charge_current() * 0.001), 3);
    // IC Temp
    OLED.setCursor(0, 41);
    OLED.print("IC");
    OLED.print(char(247));
    OLED.print("C:");
    OLED.println(ntcIC(), 1);
    // IDPM
    OLED.setCursor(68, 41);
    OLED.print("IDPM:");
    OLED.println(float(CHARGER.read_idpm_limit() * 0.001), 2);

    OLED.display();
  } else if (oled_sleep > 60) {
    OLED.ssd1306_command(SSD1306_DISPLAYOFF);

    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), wakeUp, LOW);

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW));

    OLED.ssd1306_command(SSD1306_DISPLAYON);
    oled_sleep = 0;
  }
}

void wakeUp() {
}

void timer_service() {
  ENCODER.service();
}

// The event handler for the button.
void handleEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {

  // Print out a message for all events.
  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);

  // Control the LED only for the Pressed and Released events.
  // Notice that if the MCU is rebooted while the button is pressed down, no
  // event is triggered and the LED remains off.
  switch (eventType) {
    case AceButton::kEventPressed:
      leds[0] = CRGB::Red;
      FastLED.show();
      delay(500);
      leds[0] = CRGB::Black;
      FastLED.show();
      break;
    case AceButton::kEventReleased:
      leds[1] = CRGB::Green;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Black;
      FastLED.show();
      break;
  }
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

void chargeLED() {
  stateLED0 = !stateLED0;
  FastLED.show();
  switch (CHARGER.get_charging_status()) {
    case 0:
      leds[0] = CRGB::Black;
      break;
    case 1:
      if (stateLED0) {
        leds[0] = CRGB::Black;
      } else {
        leds[0] = CRGB::DarkOrange;
      }
      break;
    case 2:
      if (stateLED0) {
        leds[0] = CRGB::Black;
      } else {
        leds[0] = CRGB::DarkRed;
      }
      break;
    case 3:
      if (stateLED0) {
        leds[0] = CRGB::Black;
      } else {
        leds[0] = CRGB::LimeGreen;
      }
      break;
  }
}