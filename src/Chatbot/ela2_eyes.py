import cv2
import mediapipe as mp
import socket
import time

# === Initialize MediaPipe Pose and Hands ===
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(model_complexity=1)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

mp_drawing = mp.solutions.drawing_utils

# === TCP Function to send events to the chatbot ===
def send_to_chatbot(event_message):
    chatbot_host = '127.0.0.1'
    chatbot_port = 12345  # Chatbot TCP listener port
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((chatbot_host, chatbot_port))
            s.sendall(event_message.encode('utf-8'))
            print(f"Sent '{event_message}' to chatbot.")
    except Exception as e:
        print(f"Error sending '{event_message}':", e)

# === Webcam Setup ===
cap = cv2.VideoCapture(6)  # Adjust index if needed
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# === Cooldown Settings (in seconds) ===
wave_cooldown = 20.0
shake_cooldown = 20.0
last_wave_time = 0
last_shake_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape

    # Process pose and hand results
    pose_results = pose.process(rgb_frame)
    hand_results = hands.process(rgb_frame)

    waving = False
    handshake = False

    # --- Pose Detection: Arms & Shoulders (for waving) ---
    if pose_results.pose_landmarks:
        landmarks = pose_results.pose_landmarks.landmark
        keypoints = {
            "left_shoulder": mp_pose.PoseLandmark.LEFT_SHOULDER,
            "right_shoulder": mp_pose.PoseLandmark.RIGHT_SHOULDER,
            "left_elbow": mp_pose.PoseLandmark.LEFT_ELBOW,
            "right_elbow": mp_pose.PoseLandmark.RIGHT_ELBOW,
            "left_wrist": mp_pose.PoseLandmark.LEFT_WRIST,
            "right_wrist": mp_pose.PoseLandmark.RIGHT_WRIST,
        }
        lm = {name: landmarks[point] for name, point in keypoints.items()}

        if all(lm[name].visibility > 0.6 for name in lm):
            shoulder_dist = abs(lm["left_shoulder"].x - lm["right_shoulder"].x)
            if 0.1 < shoulder_dist < 0.6:
                connections = [
                    (mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.LEFT_ELBOW),
                    (mp_pose.PoseLandmark.LEFT_ELBOW, mp_pose.PoseLandmark.LEFT_WRIST),
                    (mp_pose.PoseLandmark.RIGHT_SHOULDER, mp_pose.PoseLandmark.RIGHT_ELBOW),
                    (mp_pose.PoseLandmark.RIGHT_ELBOW, mp_pose.PoseLandmark.RIGHT_WRIST),
                    (mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.RIGHT_SHOULDER),
                ]
                for conn in connections:
                    start = landmarks[conn[0]]
                    end = landmarks[conn[1]]
                    x1, y1 = int(start.x * w), int(start.y * h)
                    x2, y2 = int(end.x * w), int(end.y * h)
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                for name in lm:
                    cx, cy = int(lm[name].x * w), int(lm[name].y * h)
                    cv2.circle(frame, (cx, cy), 6, (0, 255, 255), -1)
                    cv2.putText(frame, name, (cx, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                if lm["right_wrist"].y < lm["right_shoulder"].y:
                    waving = True
                elif lm["left_wrist"].y < lm["left_shoulder"].y:
                    waving = True

    # --- Hands Detection: Right hand only ---
    if hand_results.multi_hand_landmarks and hand_results.multi_handedness:
        for hand_landmarks, handedness in zip(hand_results.multi_hand_landmarks, hand_results.multi_handedness):
            label = handedness.classification[0].label  # 'Right' or 'Left'
            if label != 'Right':
                continue  # Ignore if not the right hand

            y_coords = [lm.y * h for lm in hand_landmarks.landmark]
            hand_height = max(y_coords) - min(y_coords)

            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            cv2.putText(frame, f"Right Hand height: {int(hand_height)}px",
                        (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            if hand_height > 150:
                handshake = True

    # --- Display statuses on frame ---
    cv2.putText(frame, f"Waving: {waving}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 255, 0), 2)
    cv2.putText(frame, f"Handshake: {handshake}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 200, 255), 2)

    current_time = time.time()
    if waving and (current_time - last_wave_time > wave_cooldown):
        send_to_chatbot("wave_detected")
        last_wave_time = current_time

    if handshake and (current_time - last_shake_time > shake_cooldown):
        send_to_chatbot("handshake_detected")
        last_shake_time = current_time

    cv2.imshow("Arm, Shoulder, and Hand Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

