import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

def get_path_from_file(file_path):
    path = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    x, y = float(parts[0]), float(parts[1])
                    category = parts[2] if len(parts) > 2 else '0'  # 기본값 0
                    path.append((x, y, category))
                except ValueError:
                    continue
    
    if not path:
        print("No valid coordinates found in the file.")
        return []
    
    return path

def plot_path(path, ax, options):
    label = options.get("label", "")
    plot_start = options.get("plot_start", False)
    
    # 색상 매핑
    color_map = {"": "grey", "0": "grey", "1": "blue", "2": "yellow", "3" : "green"}
    
    for x, y, category in path:
        color = color_map.get(category, "red")  # 기본적으로 빨간색
        ax.scatter(x, y, color=color, s=20)
    
    # 전체 경로를 선으로 표시
    x_vals, y_vals = zip(*[(x, y) for x, y, _ in path])
    ax.plot(x_vals, y_vals, linestyle='-', markersize=1, label=label, color='black')
    
    # 시작점 강조
    if plot_start:
        ax.scatter(x_vals[0], y_vals[0], color='red', s=50, label='Start Point (' + label + ')')

def plot_path_from_file(file_path, ax, options):
    path = get_path_from_file(file_path)
    plot_path(path, ax, options)

def main():
    fig, ax = plt.subplots(figsize=(10, 10))

    # plot rddf

    file_path_rddf = "2025-2-11_17-24_rddf-01.txt"
    options_rddf = {
        "label": "2025-2-11_17-24_rddf-01.txt",
        "plot_start": False
    }
    plot_path_from_file(file_path_rddf, ax, options_rddf)

    # plot drive case
    file_path = "2025-2-18_20-19_test-25-02-18-001_case-01.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    #plot_path_from_file(file_path, ax, options)

    file_path = "2025-2-18_20-22_test-25-02-18-001_case-02.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    plot_path_from_file(file_path, ax, options)

    file_path = "2025-2-18_20-26_test-25-02-18-001_case-03.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    #plot_path_from_file(file_path, ax, options)

    file_path = "2025-2-18_20-36_test-25-02-18-004_case-01.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    #plot_path_from_file(file_path, ax, options)

    file_path = "2025-2-18_20-40_test-25-02-18-004_case-02.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    #plot_path_from_file(file_path, ax, options)

    file_path = "2025-2-18_20-43_test-25-02-18-004_case-03.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    #plot_path_from_file(file_path, ax, options)

    file_path = "2025-2-18_20-46_test-25-02-18-001_case-04.txt"
    options = {
        "label": file_path,
        "plot_start": True
    }
    #plot_path_from_file(file_path, ax, options)
    
    # setup ax
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title("2025-2-18_20-22_test-25-02-18-001_case-02.txt")
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))  
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    #ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')

    plt.savefig("high_res_graph.png", dpi=600, bbox_inches='tight')  # 300 DPI로 저장
    plt.show()

if __name__ == "__main__":
    main()
