import numpy as np
import math

def load_coordinates(file_path):
    """텍스트 파일에서 좌표를 불러오는 함수"""
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 2:
                x, y = map(float, parts)
                coordinates.append((x, y))
    return np.array(coordinates)

def transform_coordinates(coords, translate_x=0, translate_y=0, rotation_angle=0):
    """첫 번째 좌표를 기준으로 좌표를 이동 및 회전 변환하는 함수"""
    if len(coords) == 0:
        return coords
    
    # 회전 중심점 (첫 번째 좌표)
    center_x, center_y = coords[0]
    
    # 좌표를 회전 중심 기준으로 이동
    shifted_coords = coords - np.array([center_x, center_y])
    
    # 각도를 라디안으로 변환
    angle_rad = math.radians(rotation_angle)
    cos_theta, sin_theta = math.cos(angle_rad), math.sin(angle_rad)
    
    # 회전 변환 행렬 적용
    rotated_coords = np.array([
        [cos_theta * x - sin_theta * y, sin_theta * x + cos_theta * y]
        for x, y in shifted_coords
    ])
    
    # 원래 중심점으로 되돌리고, 추가 이동 적용
    transformed_coords = rotated_coords + np.array([center_x + translate_x, center_y + translate_y])
    
    return transformed_coords

def save_coordinates(file_path, coords):
    """변환된 좌표를 새로운 파일로 저장하는 함수"""
    with open(file_path, 'w') as file:
        for x, y in coords:
            file.write(f"{x:.4f}\t{y:.4f}\n")

def main():
    input_file = "rddf_jeju2025_official.txt"  # 입력 파일 경로
    output_file = "transformed_coordinates.txt"  # 출력 파일 경로
    
    # 변환 설정
    translate_x = 66786 + 25 # X축 이동 거리
    translate_y = 469702 + 50   # Y축 이동 거리
    rotation_angle = 60  # 회전 각도 (도 단위, 반시계 방향 양수)
    
    # 좌표 로드
    coordinates = load_coordinates(input_file)
    
    # 변환 적용
    transformed_coords = transform_coordinates(coordinates, translate_x, translate_y, rotation_angle)
    
    # 변환된 좌표 저장
    save_coordinates(output_file, transformed_coords)
    print(f"변환된 좌표가 {output_file} 파일에 저장되었습니다.")

if __name__ == "__main__":
    main()
