import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

def plot_coordinates_from_file(file_path, file_path2):
    
    coordinates2 = []
    # 파일 읽기
    with open(file_path2, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    coordinates2.append((x, y))
                except ValueError:
                    continue
    
    if not coordinates2:
        print("No valid coordinates found in the file.")
        return
    
    # x, y 좌표 분리
    x_vals, y_vals = zip(*coordinates2)
    
    # 그래프 설정
    fig, ax = plt.subplots(figsize=(10, 10))
    scatter = ax.scatter(x_vals, y_vals, color='yellow', picker=True)
    ax.plot(x_vals, y_vals, linestyle='-', markersize=2, label='Tracker Path')

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
    #fig, ax = plt.subplots(figsize=(10, 10))
    scatter = ax.scatter(x_vals, y_vals, color='blue', picker=True)
    ax.plot(x_vals, y_vals, linestyle='-', markersize=2, label='RDDF')
    
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


################################################################################


def get_path_from_file(file_path):
    path = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if True:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    path.append((x, y))
                except ValueError:
                    continue
    
    if not path:
        print("No valid coordinates found in the file.")
        return
    
    return path

def plot_path(path, ax, options):
    # read options
    label = None
    if "label" in options:
        label = str(options["label"])

    color = None
    if "color" in options:
        color = options["color"]

    plot_start = False
    if "plot_start" in options:
        plot_start = options["plot_start"]

    # x, y 좌표 분리
    x_vals, y_vals = zip(*path)
    
    # 그래프 설정
    scatter = ax.scatter(x_vals, y_vals, color=color, picker=True, s=5)
    ax.plot(x_vals, y_vals, linestyle='-', markersize=2, label=label, color=color)
    
    # 시작점에 빨간 점 표시
    if plot_start:
        ax.scatter(x_vals[0], y_vals[0], color='red', s=50, label='Start Point('+label+')')
    
    return

def plot_path_from_file(file_path, ax, options):
    path = get_path_from_file(file_path)
    plot_path(path, ax, options)
    return

def main():
    fig, ax = plt.subplots(figsize=(10, 10))

    file_path_rddf = "2025-2-19_20-49_rddf-bb-01.txt"
    options_rddf = {
        "label" : "2025-2-11_17-24_rddf-01.txt",
        "color" : "black",
        "start_point" : True
    }
    plot_path_from_file(file_path_rddf, ax, options_rddf)

    file_path_rddf = "2025-2-19_21-11_rddf-bb-02_res-0.5.txt"
    options_rddf = {
        "label" : file_path_rddf,
        "color" : "grey",
        "start_point" : True
    }
    plot_path_from_file(file_path_rddf, ax, options_rddf)

    # plot drive case

    file_path_drive = "2025-2-19_20-54_bb-01_pp-01.txt"
    options_drive = {
        "label" : file_path_drive,
        "color" : "blue",
        "start_point" : False
    }
    plot_path_from_file(file_path_drive, ax, options_drive)

    file_path_drive = "2025-2-19_20-59_bb-01_pp-02.txt"
    options_drive = {
        "label" : file_path_drive,
        "color" : "cyan",
        "start_point" : False
    }
    plot_path_from_file(file_path_drive, ax, options_drive)

    file_path_drive = "2025-2-19_21-15_bb-02_pp-01.txt"
    options_drive = {
        "label" : file_path_drive,
        "color" : "red",
        "start_point" : False
    }
    plot_path_from_file(file_path_drive, ax, options_drive)

    file_path_drive = "2025-2-19_21-21_bb-02_pp-02.txt"
    options_drive = {
        "label" : file_path_drive,
        "color" : "orange",
        "start_point" : False
    }
    plot_path_from_file(file_path_drive, ax, options_drive)

    # setup ax
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('RDDF Path Plot')
    # axis tick
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))  
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')

    plt.savefig("rddf-bb-01-and-02.png", dpi=600, bbox_inches='tight')  # 300 DPI로 저장
    plt.show()
    
    return

if __name__ == "__main__":
    main()
