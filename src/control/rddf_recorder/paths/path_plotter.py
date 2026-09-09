import matplotlib.pyplot as plt
import numpy as np

def plot_coordinates_from_file(file_path):
    coordinates = []
    
    # 파일 읽기
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    coordinates.append((x, y))
                except ValueError:
                    continue
    
    if not coordinates:
        print("No valid coordinates found in the file.")
        return
    
    # x, y 좌표 분리
    x_vals, y_vals = zip(*coordinates)
    
    # 그래프 설정
    fig, ax = plt.subplots(figsize=(10, 10))
    scatter = ax.scatter(x_vals, y_vals, color='blue', picker=True)
    ax.plot(x_vals, y_vals, linestyle='-', markersize=2)
    
    # 시작점에 빨간 점 표시
    ax.scatter(x_vals[0], y_vals[0], color='red', s=50, label='Start Point')
    
    # 마우스 이벤트 핸들러 추가
    annot = ax.annotate("", xy=(0, 0), xytext=(15, 15), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)
    
    def on_pick(event):
        idx = event.ind[0]
        x, y = x_vals[idx], y_vals[idx]
        annot.xy = (x, y)
        annot.set_text(f"Idx: {idx}")
        annot.set_visible(True)
        fig.canvas.draw_idle()
    
    fig.canvas.mpl_connect("pick_event", on_pick)
    
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Coordinate Path Plot')
    ax.legend()
    ax.grid(True)
    plt.show()

# 사용 예제 (파일 경로 입력 필요)
file_path = '2025-7-29_12-44.txt'  # 실제 파일 경로로 변경
plot_coordinates_from_file(file_path)
