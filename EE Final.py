import cv2
import mediapipe
import serial
import time

def detect_fingers(frame, hand_landmarks):
    if not hand_landmarks:
        return None

    # array to track fingers raised (0 lowered, 1 raised)
    fingers = [0, 0, 0, 0, 0]  # thumb, index, middle, ring, pinky

    # fingertip points (landmarks)
    tip_ids = [4, 8, 12, 16, 20]
    
    # look at x coord instead of y for thumb detection
    if hand_landmarks.landmark[tip_ids[0]].x < hand_landmarks.landmark[tip_ids[0] - 1].x:
        fingers[0] = 1

    # other 4 fingers based on y
    for i in range(1, 5):
        if hand_landmarks.landmark[tip_ids[i]].y < hand_landmarks.landmark[tip_ids[i] - 2].y:
            fingers[i] = 1

    finger_count = sum(fingers)  # total fingers raised

    if finger_count == 0:
        return '#'  # fist submits
    elif finger_count == 5:
        return '*'  # 5 fingers clears
    else:
        return finger_count

def main():
    mp_hands = mediapipe.solutions.hands # MediaPipe hand tracking
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7) # initiliazes hand tracking
    mp_draw = mediapipe.solutions.drawing_utils # displays hand tracking
    
    seri = serial.Serial('COM3', 9600, timeout=1) 
    time.sleep(2) # wait for serial to start
    
    # initialize video capture
    capture = cv2.VideoCapture(0)
    
    # frame capture
    while capture.isOpened():
        returnstatus, frame = capture.read() # capture signal frame
        if not returnstatus:
            break # break if not working    
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # bgr (cv2) to rgb (mediapipe)
        results = hands.process(frame_rgb) # detect hand
        
        number = None

        # detects number of fingers raised if hand is detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                number = detect_fingers(frame, hand_landmarks) 
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
        # display detected fingers if valid number was detected
        if number is not None:
            cv2.putText(frame, f'Input: {number}', (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)
            seri.write(f'{number}'.encode())  # send detected number or command to Arduino
            time.sleep(1) # delay input
        
        cv2.imshow('Hand Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    capture.release()
    cv2.destroyAllWindows()
    seri.close()

if __name__ == '__main__':
    main()