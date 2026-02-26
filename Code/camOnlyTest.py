import math
import cv2
import mediapipe as mp
import time
from pyfirmata import Arduino, SERVO
from CoordinatesV5 import *

#board = Arduino("/dev/cu.usbmodem1101")
#board.digital[3].mode = SERVO
#board.digital[5].mode = SERVO
#board.digital[6].mode = SERVO
#board.digital[9].mode = SERVO

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)

cap.set(3, 300)
cap.set(4, 300)

robotArm = EEZYbotARM_Mk2(0, 0, 0)

with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.8,
    min_tracking_confidence=0.5) as hands:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    timeNow = time.time()

    if results.multi_hand_landmarks:
        time.sleep(0.016)
        for hand_landmarks in results.multi_hand_landmarks:
            
            handCoordinatesX = round(450 - hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * 500, 2)
            if handCoordinatesX < 115:
                handCoordinatesX = 115
            handCoordinatesY = round(300 - hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * 600, 2)
            handWidth = 600 * math.sqrt(
                    (hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x - 
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x)**2 +
                    (hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y - 
                    hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y)**2
                     )
            handCoordinatesZ = (round(75 * 50 / handWidth) - 20) * 3
            if handCoordinatesZ < 0:
                handCoordinatesZ = 0
            fingerDist = math.sqrt(
                    (hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x - 
                    hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x)**2 +
                    (hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y - 
                    hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y)**2
                    )
            
            grabber = (handWidth / 600) / fingerDist * 60 - 20
            print(grabber)

            if grabber < 0:
                grabber = 0
            if(grabber > 90):
                grabber = 90
            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
            print(f"Hand coordinates X Y Z: {handCoordinatesX, handCoordinatesY, handCoordinatesZ}")
            print(f"Grabber motor pos: {grabber}")
#            board.digital[3].write(grabber)
            try:
                x, y ,z = handCoordinatesX, handCoordinatesY, handCoordinatesZ
                q1, q2, q3 = EEZYbotARM_Mk2.inverseKinematics(robotArm, x, y, z)
                baseAngle, armAngle, vArmAngle, = EEZYbotARM_Mk2.map_kinematicsToServoAngles(robotArm, q1 = q1, q2 = q2, q3 = q3)
                print(baseAngle, armAngle, vArmAngle)
#                board.digital[5].write(180 - baseAngle)
#                board.digital[6].write(armAngle)
#                board.digital[9].write(vArmAngle)
            except:
                print("Robot arm cannot reach coordinates")
                pass
          
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(1) & 0xFF == ord("q"):
      break
cap.release()