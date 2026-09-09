# 카메라 내부 파라미터 계산할 때만 씀
import numpy as np
import cv2
import glob

wc = 10 # 가로 코너 수
hc = 6 # 세로 코너 수

# 종료 기준
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 체커보드의 객체 포인트 준비
objp = np.zeros((wc * hc, 3), np.float32)
objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)

# 실제 세계와 이미지 평면에서 객체 포인트와 이미지 포인트를 저장할 배열
objpoints = []  # 실제 3D 점
imgpoints = []  # 이미지 평면의 2D 점

# 켈리브레이션에 사용할 체커보드 이미지 파일 읽음.
images = glob.glob('images/*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체커보드 코너 찾음.
    ret, corners = cv2.findChessboardCorners(gray, (wc, hc), None)

    # 코너를 찾았으면 객체 포인트와 이미지 포인트 저장함.
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # 코너 그려서 보여줌.
        img = cv2.drawChessboardCorners(img, (wc, hc), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print('내부 파라미터 행렬:', mtx)
print('왜곡 계수 :', dist) # 요즘은 카메라가 잘 나와서 0으로 하면됨.