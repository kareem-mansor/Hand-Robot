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
        ser.write(f'${finger_data}\n'.encode())  # Send data to Arduino
        cv2.imshow("Image", img)
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    main()
