

file_path = "2025-3-9_17-34_dv-02.txt"

# 파일을 한 줄씩 읽어와서 공백을 적절히 처리하여 데이터 변환
with open(file_path, 'r', encoding='utf-8') as file:
    lines = file.readlines()

# 각 줄을 공백 기준으로 나누고 첫 번째 열을 제외한 x, y 좌표만 추출
filtered_data = []
for line in lines:
    parts = line.split()
    filtered_data.append(f"{parts[0]}\t{parts[1]}")

# 변환된 데이터를 새로운 파일에 저장
output_path = "filtered_" + file_path
with open(output_path, 'w', encoding='utf-8') as file:
    file.write("\n".join(filtered_data))

# 변환된 파일 경로 출력
output_path
