#include <Arduino.h>
#include <EEPROM.h>
#include <TM1637TinyDisplay6.h>
#include <RTClib.h>

#define CLK_PIN 2
#define DIO_PIN 3
#define SAB6456_PIN 6
#define RELAY_PIN 7
#define BUTTON_PIN A0
#define CALIBRATION_PIN A1

TM1637TinyDisplay6 display(CLK_PIN, DIO_PIN);
RTC_DS1307 rtc;

unsigned int c_init = 0;
unsigned int counts = 0;
unsigned int counts_store = 0;
byte index = 0;

// these are checked for in the main program
volatile unsigned long timerCounts;
volatile boolean counterReady;

// internal to counting routine
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;

const int MODE1 = 0;
const int MODE64 = 1;
const int MODE256 = 2;
int mode = MODE1;
int conditionCount = 0;

const int USER_MODE1 = MODE1;
const int USER_MODE64 = MODE64;
const int USER_MODE256 = MODE256;
const int USER_AUTO = 4;
int userMode = USER_AUTO;

bool isCalibrating = false;
const float CALIBRATE_ERROR = 0.05;
float calibrateFactor = 1;

unsigned long long displayTimeout = 0;

void startCounting(unsigned int ms) {
  counterReady = false;  // time not up yet
  timerPeriod = ms;      // how many 1 ms counts to do
  timerTicks = 0;        // reset interrupt counter
  overflowCount = 0;     // no overflows yet

  // reset Timer 1 and Timer 2
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer 1 - counts events on pin D5
  TIMSK1 = bit(TOIE1);  // interrupt on Timer 1 overflow

  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs.
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit(WGM21);  // CTC mode
  OCR2A = 124;          // count up to 125  (zero relative!!!!) // def = 124

  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit(OCIE2A);  // enable Timer2 Interrupt

  TCNT1 = 0;  // Both counters to zero
  TCNT2 = 0;

  // Reset prescalers
  GTCCR = bit(PSRASY);  // reset prescaler now
  // start Timer 2
  TCCR2B = bit(CS20) | bit(CS22);  // prescaler of 128
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B = bit(CS10) | bit(CS11) | bit(CS12);
}  // end of startCounting

ISR(TIMER1_OVF_vect) {
  ++overflowCount;  // count number of Counter1 overflows
}  // end of TIMER1_OVF_vect

//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR(TIMER2_COMPA_vect) {
  // grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;

  // see if we have reached timing period
  if (++timerTicks < timerPeriod)
    return;  // not yet

  // if just missed an overflow
  if ((TIFR1 & bit(TOV1)) && timer1CounterValue < 256)
    overflowCopy++;

  // end of gate time, measurement ready

  TCCR1A = 0;  // stop timer 1
  TCCR1B = 0;

  TCCR2A = 0;  // stop timer 2
  TCCR2B = 0;

  TIMSK1 = 0;  // disable Timer1 Interrupt
  TIMSK2 = 0;  // disable Timer2 Interrupt

  // calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more
  counterReady = true;                                      // set global flag for end count period
}  // end of TIMER2_COMPA_vect

int getCountingMillis() {
  switch (mode) {
    case MODE1: return 500;
    default: return 82;
  }
}

float getValue(float frq) {
  switch (mode) {
    case MODE1: return frq;
    case MODE64: return frq * 64;
    case MODE256: return frq * 256;
  }
}

int getRetryCount() {
  switch (mode) {
    case MODE1: return 2;
    default: return 5;
  }
}

void checkMode(float value) {
  if (userMode != USER_AUTO) {
    if (userMode != mode) {
      switchMode(userMode);
    }
    return;
  }

  switch (mode) {
    case MODE1:
      if (value > 1000000) {
        conditionCount++;
        if (conditionCount >= getRetryCount()) {
          switchMode(MODE64);
        }
      } else {
        conditionCount = 0;
      }
      break;
    case MODE64:
      if (value < 500000) {
        conditionCount++;
        if (conditionCount >= getRetryCount()) {
          switchMode(MODE1);
        }
      } else if (value > 64000000) {
        conditionCount++;
        if (conditionCount >= getRetryCount()) {
          switchMode(MODE256);
        }
      } else {
        conditionCount = 0;
      }
      break;
    case MODE256:
      if (value < 32000000) {
        conditionCount++;
        if (conditionCount >= getRetryCount()) {
          switchMode(MODE64);
        }
      } else {
        conditionCount = 0;
      }
      break;
  }
}

void switchMode(int newMode) {
  switch (newMode) {
    case MODE1:
      pinMode(RELAY_PIN, INPUT);
      pinMode(SAB6456_PIN, INPUT);
      break;
    case MODE64:
      pinMode(RELAY_PIN, OUTPUT);
      digitalWrite(RELAY_PIN, LOW);
      pinMode(SAB6456_PIN, INPUT);
      break;
    case MODE256:
      pinMode(RELAY_PIN, OUTPUT);
      digitalWrite(RELAY_PIN, LOW);
      pinMode(SAB6456_PIN, OUTPUT);
      digitalWrite(SAB6456_PIN, LOW);
      break;
  }
  mode = newMode;
}

void switchUserMode() {
  switch (userMode) {
    case USER_AUTO:
      userMode = USER_MODE1;
      display.showString("    E1");
      break;
    case USER_MODE1:
      userMode = USER_MODE64;
      display.showString("   E64");
      break;
    case USER_MODE64:
      userMode = USER_MODE256;
      display.showString("  E256");
      break;
    case USER_MODE256:
      userMode = USER_AUTO;
      display.showString("  AUTO");
      break;
  }
  displayTimeout = millis() + 500;
}

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(CALIBRATION_PIN, INPUT);
  rtc.begin();
  rtc.writeSqwPinMode(DS1307_OFF);
  display.setBrightness(4);
  display.showNumber(888888);
  switchMode(MODE1);
  EEPROM.get(0, calibrateFactor);
  if (isnan(calibrateFactor) || abs(calibrateFactor - 1) > CALIBRATE_ERROR) {
    calibrateFactor = 1;
  }
}

void loop() {
  if (millis() < 1000) return;
  if (millis() < displayTimeout) return;

  bool isButton = digitalRead(BUTTON_PIN) == LOW;
  if (isButton) {
    switchUserMode();
    return;
  }

  bool isCalibration = digitalRead(CALIBRATION_PIN) == LOW;
  if (isCalibration && !isCalibrating) {
    switchMode(MODE1);
    rtc.writeSqwPinMode(DS1307_SquareWave32kHz);
    calibrateFactor = -1;
    isCalibrating = true;
  }
  if (!isCalibration && isCalibrating) {
    rtc.writeSqwPinMode(DS1307_OFF);
    isCalibrating = false;
  }

  PORTB |= _BV(PB4);  // For benchmarking purposes

  // stop Timer 0 interrupts from throwing the count out
  byte oldTCCR0A = TCCR0A;
  byte oldTCCR0B = TCCR0B;
  TCCR0A = 0;  // stop timer 0
  TCCR0B = 0;

  int countingMillis = getCountingMillis();
  startCounting(countingMillis);  // 42 //how many ms to count for //def 500

  while (!counterReady) {
  }  // loop until count over

  // adjust counts by counting interval to give frequency in Hz
  float frq = timerCounts * 1000.0 / countingMillis;  // / timerPeriod;
  if (isCalibration) {
    float factor = 32768 / frq;
    if (abs(factor - 1) < CALIBRATE_ERROR && calibrateFactor < 0) {
      EEPROM.put(0, factor);
      calibrateFactor = factor;
    }
  } else {
    frq *= calibrateFactor;
  }
  float value = getValue(frq);

  if (value < 1000000) {
    display.showNumber((long)(value));
  } else {
    display.showNumber(value / 1000000, 3, MAXDIGITS);
  }

  if (!isCalibration) {
    checkMode(value);
  }

  // restart timer 0
  TCCR0A = oldTCCR0A;
  TCCR0B = oldTCCR0B;
}  // end of loop