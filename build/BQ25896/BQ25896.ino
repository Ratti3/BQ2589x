#include <Wire.h>
#include <BQ2589x.h>
#include <FastLED.h>
#include <TimerOne.h>
#include <LowPower.h>
#include <BasicEncoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// FastLED Inputs
#define LED_BRIGHTNESS 10
// 2 = Number of LEDs
CRGB leds[2];

// BQ25896 Address
#define BQ25895 0x6B

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

  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);

  FastLED.addLeds<WS2812, 11, RGB>(leds, 2);
  FastLED.setBrightness(LED_BRIGHTNESS);

  delay(250);
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  OLED.clearDisplay();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer_service);
  ENCODER.set_reverse();

  CHARGER.begin(&Wire);
  CHARGER.adc_start(0);
  CHARGER.enable_otg();
}

void loop() {

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

    //reset watch dog
    CHARGER.reset_watchdog_timer();

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
    OLED.print("TEMP: ");
    OLED.println(CHARGER.adc_read_temperature());
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
    OLED.display();
    delay(1000);

    Serial.println(CHARGER.adc_read_charge_current());  // read charge current.
  }
}

void timer_service() {
  ENCODER.service();
}