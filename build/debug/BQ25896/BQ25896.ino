#include <Wire.h>
#include <BQ2589x.h>
#include <FastLED.h>
#include <TimerOne.h>
#include <LowPower.h>
#include <AceButton.h>
#include <BasicEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

using namespace ace_button;
const int ENCODER_SW = A5;
#define LONGPRESSDURATION 5000
AceButton button(ENCODER_SW);
void handleEvent(AceButton*, uint8_t, uint8_t);

// FastLED Inputs
#define LED_BRIGHTNESS 10
// 2 = Number of LEDs
CRGB leds[2];

// OLED Inputs
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 OLED(128, 64, &Wire, -1);

// Rotary Encoder Inputs
BasicEncoder ENCODER(A3, A4);

bq2589x CHARGER;
unsigned long last_change = 0;
unsigned long now = 0;

void setup() {

  Wire.begin();
  Serial.begin(9600);

  // Wait for Serial
  delay(2000);
  Serial.println("Setup");
  
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);

  pinMode(ENCODER_SW, INPUT);
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  buttonConfig->setLongPressDelay(LONGPRESSDURATION);

  FastLED.addLeds<WS2812, 11, RGB>(leds, 2);
  FastLED.setBrightness(LED_BRIGHTNESS);

  delay(250);
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  OLED.clearDisplay();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_service);
  ENCODER.set_reverse();

  CHARGER.begin(&Wire);
  Serial.println("Dis WDT");
  CHARGER.disable_watchdog_timer();
  Serial.println("ADC Start");
  CHARGER.adc_start(0);
  Serial.println("CRG V");
  CHARGER.set_chargevoltage(4192);
  Serial.println("EN OTG");
  CHARGER.enable_otg();
  Serial.println("OTG V");
  CHARGER.set_otg_volt(4998);
  Serial.println("OTG I");
  CHARGER.set_otg_current(2150);
}

void loop() {

  button.check();

  /*
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Blue;
  FastLED.show();
  delay(50);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Green;
  leds[1] = CRGB::Orange;
  FastLED.show();
  delay(50);
*/

  int encoder_change = ENCODER.get_change();
  if (encoder_change) {
    Serial.println(ENCODER.get_count());
  }

  //OLED.ssd1306_command(SSD1306_DISPLAYOFF);
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  //OLED.ssd1306_command(SSD1306_DISPLAYON);

  now = millis();

  if (now - last_change > 5000) {
    last_change = now;

    OLED.clearDisplay();
    OLED.setTextSize(1);
    OLED.setTextColor(SSD1306_WHITE);
    OLED.setCursor(0, 0);
    OLED.print("CHRG: ");
    switch (CHARGER.get_charging_status())  // charger status
    {
      case 0:
        OLED.println("Not Charging");
        break;
      case 1:
        OLED.println("Pre-charge");
        break;
      case 2:
        OLED.println("Fast Charging");
        break;
      case 3:
        OLED.println("Charge Done");
        break;
    }
    OLED.setCursor(0, 10);
    OLED.print("TYPE: ");
    OLED.println(CHARGER.get_vbus_type());
    OLED.setCursor(0, 20);
    OLED.print("VSYS: ");
    OLED.println(CHARGER.adc_read_sys_volt());
    OLED.setCursor(0, 30);
    OLED.print("VBAT: ");
    OLED.println(CHARGER.adc_read_battery_volt());
    OLED.setCursor(0, 40);
    OLED.print("VBUS: ");
    OLED.println(CHARGER.adc_read_vbus_volt());
    OLED.setCursor(0, 50);
    OLED.print("IDPM: ");
    OLED.println(CHARGER.read_idpm_limit());

    //Serial.println(CHARGER.adc_read_charge_current());  // read charge current.
    OLED.setCursor(70, 20);
    OLED.print("WDT: ");
    OLED.println(CHARGER.get_fault_status(7));
    OLED.setCursor(70, 30);
    OLED.print("OTG: ");
    OLED.println(CHARGER.get_fault_status(6));
    OLED.setCursor(70, 40);
    OLED.print("CRG: ");
    OLED.println(CHARGER.get_fault_status(3));
    Serial.print("WDT Fault:");Serial.println(CHARGER.get_fault_status(7));
    Serial.print("OTG Fault:");Serial.println(CHARGER.get_fault_status(6));
    Serial.print("CRG Fault:");Serial.println(CHARGER.get_fault_status(4));
    Serial.print("BAT Fault:");Serial.println(CHARGER.get_fault_status(3));
    Serial.print("NTC Fault:");Serial.println(CHARGER.get_fault_status(0));

    OLED.display();
  }
}

void timer_service() {
  ENCODER.service();
}

// The event handler for the button.
void handleEvent(AceButton* /* button */, uint8_t eventType,
    uint8_t buttonState) {

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