import cv2

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"좌표 : ({x}, {y})")

cap = cv2.VideoCapture(0)

while cap.isOpened():
    _, frame = cap.read()

    cv2.imshow('frame', frame)
    cv2.setMouseCallback('frame', click_event)
    if cv2.waitKey(25) == ord('q'):
        break