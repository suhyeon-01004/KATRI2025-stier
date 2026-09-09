import cv2
from paddleocr import PaddleOCR, draw_ocr
import numpy as np

# PaddleOCR 초기화
ocr = PaddleOCR(use_angle_cls=True, lang='en')  # English OCR

# 카메라 초기화
cap = cv2.VideoCapture(4)

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 영상을 가져올 수 없습니다.")
        break

    try:
        # PaddleOCR로 텍스트 인식
        result = ocr.ocr(frame, cls=True)

        # 결과가 None이 아니고, 결과가 있는 경우 처리
        if result is not None and len(result[0]) > 0:
            for line in result[0]:
                # 인식된 텍스트와 위치 정보 가져오기
                box = line[0]  # 텍스트의 바운딩 박스 좌표
                text = line[1][0]  # 인식된 텍스트
                score = line[1][1]  # 인식 신뢰도

                # 바운딩 박스 그리기
                box = np.array(box).astype(np.int32)
                cv2.polylines(frame, [box], isClosed=True, color=(0, 255, 0), thickness=2)

                # 인식된 텍스트를 바운딩 박스의 왼쪽 상단에 표시
                text_position = (box[0][0], box[0][1] - 10)  # 왼쪽 상단 좌표, 상단으로 약간 올림
                cv2.putText(frame, f'{text}', text_position,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

    except Exception as e:
        print(f"오류 발생: {e}")

    # 화면에 프레임 보여주기
    cv2.imshow('Camera', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라와 윈도우 해제
cap.release()
cv2.destroyAllWindows()
