# 파일 경로 설정
input_file = "backward_rddf_stier2_1.txt"
output_file = "backward_rddf_stier2_1_reverse.txt"

# 파일 읽기 및 역순 정렬
with open(input_file, "r") as f:
    lines = f.readlines()

# 역순으로 저장
with open(output_file, "w") as f:
    f.writelines(reversed(lines))

print(f"역순으로 정렬된 파일이 저장됨: {output_file}")
