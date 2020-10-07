#include <AccelStepper.h>
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>

#define LED_STRIP 24
#define LED_PIN 10

#define MICROPHONE_PIN A0
#define ENDSTOP_PIN1 7
#define ENDSTOP_PIN2 8
#define BUTTON_PIN 9

#define RAILLENGTH 1650.00 //physical length of the rail in mm, this must be measured between the 2 endstops

AccelStepper rail(1, 3, 6); // pin 3 = step, pin 6 = direction

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_STRIP, LED_PIN, NEO_GRB + NEO_KHZ800);

Bounce startButton = Bounce();

long endPosition;
float stepsPerMM;

long startPrintPosition;
long endPrintPosition;

int mode = 0;  // 0=calibrate, 1=idle, 2=record, 3=run

int soundSampleArray[200];
uint16_t amountOfSoundSamples;
uint16_t calDif;
uint16_t calMid;
uint32_t soundLightArray[200][24];

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void buildSoundArray () {
  for (int z = 0; z < amountOfSoundSamples; z++) {
    int rainbowPos = map(z, 0, amountOfSoundSamples, 0, 255);

    int outputValue = map(soundSampleArray[z], calMid - calDif, calMid + calDif, 0, strip.numPixels());

    for (uint16_t i = 0; i < strip.numPixels(); i++) {
      if (i == outputValue) {
        strip.setPixelColor(i, Wheel(rainbowPos & 255));
      } else if (i < 12 + abs(12 - outputValue) && i > 12 - abs(12 - outputValue)) {
        strip.setPixelColor(i, 40, 30, 40);
      } else {
        strip.setPixelColor(i, 0, 0, 0);
      }
    }
  }
}

void readSoundWaveSample () {
  Serial.println("start reading");
  int maxR = 0;
  int minR = 4096;
  long totalCal = 0;
  int calRead;
  int counter = 0;
  elapsedMicros recordingTime = 0;
  while (recordingTime < 2915) {
    calRead = analogRead(MICROPHONE_PIN);
    soundSampleArray[counter] = calRead;

    if (calRead > maxR) {
      maxR = calRead;
    }
    if (calRead < minR) {
      minR = calRead;
    }
    totalCal = totalCal + calRead;

    counter++;
    delayMicroseconds(15);
  }
  calDif = ((maxR - minR) / 2);
  calMid = totalCal / counter;

  Serial.println("done reading");
  amountOfSoundSamples = counter;
}

void setNeoPixelStick (uint8_t red, uint8_t green, uint8_t blue) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, red, green, blue);
  }
  strip.show();
}

void setNeoPixel (uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i == pixel) {
      strip.setPixelColor(i, red, green, blue);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
  strip.show();
}

void setNeoPixel (uint8_t pixel, uint32_t color) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i == pixel) {
      strip.setPixelColor(i, color);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
  strip.show();
}

void resetSoundwaveSampleArray() {
  for (int z = 0; z < 200; z++) {
    soundSampleArray[z] = 0;
  }
}

void setup() {
  Serial.begin(115200);
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
    }*/
  Serial.println("////Wave length printer //////");
  
  rail.setMaxSpeed(100000);
  rail.setAcceleration(80000);

  analogReadResolution(12);

  strip.begin();
  strip.show();

  pinMode(ENDSTOP_PIN1, INPUT_PULLUP);
  pinMode(ENDSTOP_PIN2, INPUT_PULLUP);

  startButton.attach(BUTTON_PIN, INPUT_PULLUP);
  startButton.interval(10);

  pinMode(MICROPHONE_PIN, INPUT);

  bool go = true;
  while (go) {
    startButton.update();
    if (startButton.changed()){
      if (startButton.fell()) {
          go = false;
      }
    }
  }
}

void loop() {
  startButton.update();
  //  if not calibrated
  if (mode == 0) {
    Serial.println("start calibration");
    // calibrate, move rail slow till endstop is triggered
    rail.setSpeed(-100);
    while (digitalRead(ENDSTOP_PIN1) == HIGH) {
      rail.runSpeed();
    }

    delay(200);

    // set position 0
    rail.setCurrentPosition(0);

    // move rail slow other direction
    rail.setSpeed(100);
    while (digitalRead(ENDSTOP_PIN2) == HIGH) {
      rail.runSpeed();
    }
    
    // set end position
    endPosition = rail.currentPosition();

    // calculate margins/ distance to steps
    stepsPerMM = endPosition/RAILLENGTH;

    startPrintPosition = ((RAILLENGTH - 1000)/2)* stepsPerMM;
    endPrintPosition = startPrintPosition + (1000 * stepsPerMM);

    // go to start position
    rail.moveTo(20);
    rail.runToPosition(); // blocking code

    mode = 1;
    setNeoPixelStick(0, 0, 0);
    Serial.println("done calibration");
    Serial.println("Calibration parameters:");
    Serial.print("endPosition:");
    Serial.println(endPosition);
    Serial.print("stepsPerMM:");
    Serial.println(stepsPerMM);
    Serial.print("startPrintPosition:");
    Serial.println(startPrintPosition);
    Serial.print("endPrintPosition:");
    Serial.println(endPrintPosition);
  }

  // If button pushed while idle
  if (startButton.changed() && mode == 1){
    if (startButton.fell()) {
        mode = 2;
    }
  }

  // record sound
  if (mode == 2) {
    resetSoundwaveSampleArray();
    readSoundWaveSample ();
    buildSoundArray();
    mode = 3;
    delay(1000);
  }

  if (mode == 3) {
    Serial.println("start printing");
    setNeoPixelStick(0, 0, 0);
    // calculate amount of steps between start en end
    long totalAmountOfStepsToShow = endPrintPosition - startPrintPosition;
    int stepsBetweenShow = totalAmountOfStepsToShow/amountOfSoundSamples;
    long nextPrintPosition = startPrintPosition;
    int soundPosition = 0;

    rail.moveTo(endPosition - 20);
    
    while(rail.distanceToGo() != 0) {
      rail.run();
      int currentPos = rail.currentPosition();
      if (currentPos > nextPrintPosition && currentPos < endPrintPosition) {
        int rainbowPos = map(soundPosition, 0, amountOfSoundSamples, 0, 255);

        int outputValue = map(soundSampleArray[soundPosition], calMid - calDif, calMid + calDif, 0, strip.numPixels());

        for (uint16_t i = 0; i < strip.numPixels(); i++) {
          if (i == outputValue) {
            strip.setPixelColor(i, Wheel(rainbowPos & 255));
          } else if (i < 12 + abs(12 - outputValue) && i > 12 - abs(12 - outputValue)) {
            strip.setPixelColor(i, 40, 30, 40);
          } else {
            strip.setPixelColor(i, 0, 0, 0);
          }
        }
        strip.show();
        nextPrintPosition = nextPrintPosition + stepsBetweenShow;
        soundPosition++;
      } else if ( currentPos > endPrintPosition && currentPos < endPrintPosition +2 ) {
        setNeoPixelStick(0, 0, 0);
      }
    }
    
    nextPrintPosition = endPrintPosition;
    soundPosition = amountOfSoundSamples -1;
    rail.moveTo(20);
    
    while(rail.distanceToGo() != 0) {
      rail.run();
      int currentPos = rail.currentPosition();
      if (currentPos < nextPrintPosition && currentPos > startPrintPosition) {
        int rainbowPos = map(soundPosition, 0, amountOfSoundSamples, 0, 255);

        int outputValue = map(soundSampleArray[soundPosition], calMid - calDif, calMid + calDif, 0, strip.numPixels());

        for (uint16_t i = 0; i < strip.numPixels(); i++) {
          if (i == outputValue) {
            strip.setPixelColor(i, Wheel(rainbowPos & 255));
          } else if (i < 12 + abs(12 - outputValue) && i > 12 - abs(12 - outputValue)) {
            strip.setPixelColor(i, 40, 30, 40);
          } else {
            strip.setPixelColor(i, 0, 0, 0);
          }
        }
        strip.show();
        nextPrintPosition = nextPrintPosition - stepsBetweenShow;
        soundPosition--;
      } else if (currentPos < startPrintPosition && currentPos > startPrintPosition - 2) {
        setNeoPixelStick(0, 0, 0);
      }
    }
    mode = 1;
    Serial.println("done printing");
  }
}
