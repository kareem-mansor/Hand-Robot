# Hand Gesture Controlled Servo Motors

This project demonstrates the use of hand gestures to control servo motors connected to an Arduino. The system uses OpenCV and MediaPipe for hand gesture recognition and sends the data to the Arduino via serial communication.

## Arduino Code

The Arduino code reads the serial data sent from the Python script and controls the servos based on the received hand gesture data.

### `Arduino.ino`

```cpp
#include <Servo.h>

#define numOfValsRec 5
#define digitsPerValRec 1

Servo servoThumb;
Servo servoIndex;
Servo servoMiddel;
Servo servoRing;
Servo servoPinky;
int valsRec[numOfValsRec];

int stringLength = numOfValsRec * digitsPerValRec + 1;  //$00000
int counter = 0;
bool counterStart = false;
String receivedString;

void setup() {
  Serial.begin(9600);
  servoThumb.attach(7);
  servoIndex.attach(9);
  servoMiddel.attach(11);
  servoRing.attach(8);
  servoPinky.attach(10);
}

void receiveData() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '$') {
      counterStart = true;
    }
    if (counterStart) {
      if (counter < stringLength) {
        receivedString = String(receivedString + c);
        counter++;
      }
      if (counter >= stringLength) {
        for (int i = 0; i < numOfValsRec; i++) {
          int num = (i * digitsPerValRec) + 1;
          valsRec[i] = receivedString.substring(num, num + digitsPerValRec).toInt();
        }
        receivedString = "";
        counter = 0;
        counterStart = false;
      }
    }
  }
}

void loop() {
  receiveData();
  if (valsRec[0] == 1) {
    servoThumb.write(180);
  } else {
    servoThumb.write(0);
  }
  if (valsRec[1] == 1) {
    servoIndex.write(180);
  } else {
    servoIndex.write(0);
  }
  if (valsRec[2] == 1) {
    servoMiddel.write(180);
  } else {
    servoMiddel.write(0);
  }
  if (valsRec[3] == 1) {
    servoRing.write(180);
  } else {
    servoRing.write(0);
  }
  if (valsRec[4] == 1) {
    servoPinky.write(180);
  } else {
    servoPinky.write(0);
  }
}

```

## Python Code

The Python code captures video input, detects hand gestures using MediaPipe, and sends the detected gesture data to the Arduino.

### `hand_gesture_control.py`

```python
import cv2
import serial
import mediapipe as mp

# Initialize serial communication with Arduino
ser = serial.Serial('COM3', 9600, timeout=1)  # Adjust COM port as necessary

class HandDetector:
    def __init__(self, staticMode=False, maxHands=2, modelComplexity=1, detectionCon=0.5, minTrackCon=0.5):
        self.staticMode = staticMode
        self.maxHands = maxHands
        self.modelComplexity = modelComplexity
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.staticMode,
                                        max_num_hands=self.maxHands,
                                        model_complexity=modelComplexity,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]

    def findHands(self, img, draw=True, flipType=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        handFingers = [0, 0, 0, 0, 0]  # Initialize all fingers as down
        if self.results.multi_hand_landmarks:
            handLms = self.results.multi_hand_landmarks[0]  # Consider only the first detected hand
            for id, lm in enumerate(handLms.landmark):
                if id in self.tipIds:
                    if id == self.tipIds[0]:  # Special condition for the thumb
                        if lm.x < handLms.landmark[self.tipIds[0] - 1].x:  # Check if the thumb tip is to the left of the adjacent landmark
                            handFingers[0] = 1  # Set the thumb as raised
                    else:
                        if lm.y < handLms.landmark[id - 1].y:  # Check if the finger tip is above the adjacent landmark
                            handFingers[self.tipIds.index(id)] = 1  # Set the finger as raised
            if draw:
                self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return handFingers, img

def main():
    cap = cv2.VideoCapture(0)
    detector = HandDetector(staticMode=False, maxHands=2, modelComplexity=1, detectionCon=0.5, minTrackCon=0.5)
    while True:
        success, img = cap.read()
        fingers, img = detector.findHands(img, draw=True, flipType=True)
        # Convert finger states to string format and send to Arduino
        finger_data = ''.join(map(str, fingers))
        ser.write(f'${finger_data}\\n'.encode())  # Send data to Arduino
        cv2.imshow("Image", img)
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    main()

```

## Getting Started

### Hardware Requirements

- Arduino Uno
- 5 Servo Motors
- Jumper Wires
- Breadboard

### Software Requirements

- Arduino IDE
- Python 3.x
- OpenCV
- MediaPipe
- PySerial

### Setup

1. **Arduino Setup:**
    - Connect the servo motors to the Arduino as per the pin configuration in the `Arduino.ino` file.
    - Upload the `Arduino.ino` code to the Arduino board.
2. **Python Setup:**
    - Install the required Python libraries:
        
        ```bash
        pip install opencv-python mediapipe pyserial
        
        ```
        
    - Adjust the serial port in `hand_gesture_control.py` as necessary.
    - Run the Python script:
        
        ```bash
        python hand_gesture_control.py
        
        ```
        

### Usage

- Run the Python script to start the hand gesture recognition.
- Make different hand gestures to control the servos connected to the Arduino.
- Press `q` to quit the application.

Feel free to contribute to this project by submitting issues and pull requests on GitHub.
