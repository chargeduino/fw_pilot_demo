#include <Adafruit_RGBLCDShield.h>

#define USE_SERIAL 0
#define USE_RGBLCD 1

// fixed parameters
// micro-controller
#define ANALOG_VREF         5
#define ADC_MAX_VALUE_F     1023.0
#define PWM_MAX_VALUE_U     255
#define PWM_8AMP            34
// LCD: refresh rate
#define RGB_LCD_REFRESH_MS  500
// LCD: backlight colors
#define RGB_LCD_RED         0x1
#define RGB_LCD_YELLOW      0x3
#define RGB_LCD_GREEN       0x2
#define RGB_LCD_TEAL        0x6
#define RGB_LCD_BLUE        0x4
#define RGB_LCD_VIOLET      0x5
#define RGB_LCD_WHITE       0x7

// pin definitions
#define PIN_PWM             6
#define PIN_RELAY           10
#define PIN_PILOT_READ      (A0)

// type: state (for the state-machine)
typedef enum {
  IDLE,
  CONNECTED,
  CHARGING,
  FINISHED
} t_state;

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

void setup()
{
#if USE_SERIAL
  while(!Serial){;}
  Serial.begin(115200); // UART debug
#endif

#if USE_RGBLCD
  lcd.begin(16, 2); // LCD 16x2
  lcd.clear(); // also sets position to zero
#endif

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  analogReference(DEFAULT); // https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
  pinMode(PIN_PWM, OUTPUT);
  analogWrite(PIN_PWM, PWM_MAX_VALUE_U);
}

void loop()
{
  static unsigned long prevTime = 0;
  static t_state status;
  static unsigned int pwmDuty;
  static float pilotVoltageShown = 0;
  float pilotVoltage;
  float pilotVoltageScaled;
  
  pilotVoltage = analogRead(PIN_PILOT_READ) * ANALOG_VREF / ADC_MAX_VALUE_F;
  pilotVoltageScaled = pilotVoltage * 6.380200861 - 12.19512195;
  if (pilotVoltageScaled >= 0) pilotVoltageShown = pilotVoltageScaled;
#if USE_SERIAL
  Serial.println(pilotVoltage);
#endif

  if (millis() - prevTime > RGB_LCD_REFRESH_MS) {
    prevTime = millis();

    switch (status) {
      default:
        status = IDLE;
      case IDLE:
        pwmDuty = PWM_MAX_VALUE_U;
        digitalWrite(PIN_RELAY, LOW);
  #if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_VIOLET);
        lcd.print("PLUG THE VEHICLE");
        lcd.setCursor(0,1); // second row
        lcd.print("Pilot: ");
        lcd.print(pilotVoltageShown);
  #endif
        if (3.55 >= pilotVoltage && pilotVoltage > 3.08) { // connected
          status = CONNECTED;
        }
        break;
        
      case CONNECTED:
        pwmDuty = PWM_8AMP;
        digitalWrite(PIN_RELAY, LOW);
  #if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_YELLOW);
        lcd.print("CAR CONNECTED");
        lcd.setCursor(0,1); // second row
        lcd.print("Pilot: ");
        lcd.print(pilotVoltageShown);
  #endif
        if (pilotVoltage > 3.55) { // disconnected
          status = IDLE;
        }
        else if (3.08 >= pilotVoltage && pilotVoltage > 2.61) { // charge start requested
          status = CHARGING;
        }
        break;
      
      case CHARGING:
        pwmDuty = PWM_8AMP;
        digitalWrite(PIN_RELAY, HIGH);
  #if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_RED);
        lcd.print("CAR CHARGING...");
        lcd.setCursor(0,1); // second row
        lcd.print("Pilot: ");
        lcd.print(pilotVoltageShown);
  #endif
        if (3.55 >= pilotVoltage && pilotVoltage > 3.08) { // charge stop requested
          status = FINISHED;
        }
        else if (pilotVoltage > 3.55) { // disconnected
          status = IDLE;
        }
        break;
      
      case FINISHED:
        // pwmDuty ?
        digitalWrite(PIN_RELAY, LOW);
  #if USE_RGBLCD
        lcd.clear(); // also sets position to zero
        lcd.setBacklight(RGB_LCD_GREEN);
        lcd.print("CHARGE FINISHED");
        lcd.setCursor(0,1); // second row
        lcd.print("Pilot: ");
        lcd.print(pilotVoltageShown);
  #endif
        if (pilotVoltage > 3.55) { // disconnected
          status = IDLE;
        }
        break;

      // if (2.61 >= pilotVoltage) { // undetermined
    }
  
  }

  analogWrite(PIN_PWM, pwmDuty);
}
