import cv2

def main():
    cap = cv2.VideoCapture(1) # 카메라 장치를 열어 캡쳐 객체를 생성한다.

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    save_idx = 1 # 저장할 이미지의 인덱스를 초기화함.

    while True:
        ret, frame = cap.read() # 웹캠에서 frame 변수에 이미지를 저장함

        cv2.imshow('Camera', frame) # 이미지 뜸

        key = cv2.waitKey(1) & 0xFF # 키보드 입력을 대기함
        if key == ord('s'):
            filename = 'images/image_{}.png'.format(save_idx) # 파일 이름을 지정함
            save_idx += 1
            cv2.imwrite(filename, frame)
            print('saved!')
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()