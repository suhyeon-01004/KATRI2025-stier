import os
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.patches as patches


def get_path_from_file(file_path):
    path = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) == 2:
                try:
                    x, y = map(float, parts)
                    path.append((x, y))
                except ValueError:
                    continue
    return path


def plot_path(path, ax, label=None, color=None, plot_start=False, only_line=False):
    x_vals, y_vals = zip(*path)
    alpha = 0 if only_line else 1
    ax.scatter(x_vals, y_vals, color=color, s=20, alpha=alpha)
    ax.plot(x_vals, y_vals, linestyle='-', color=color, label=label)
    if plot_start:
        ax.scatter(x_vals[0], y_vals[0], color='red', s=50, label=f"Start ({label})")


def setup_plot(ax, title="RDDF Lane Change Visualization"):
    ax.set_xlabel("X Coordinate [m]")
    ax.set_ylabel("Y Coordinate [m]")
    ax.set_title(title)
    ax.legend(loc='upper left', fontsize='small')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.ticklabel_format(useOffset=True, style='plain', axis='x')
    ax.ticklabel_format(useOffset=False, style='plain', axis='y')
    
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right")


def parse_coords_and_lines(file_path):
    """(x, y) 좌표와 원본 txt의 라인 번호(1-based)를 함께 반환"""
    coords, lines = [], []
    with open(file_path, 'r') as f:
        for i, line in enumerate(f, 1):
            parts = line.strip().split()
            if len(parts) == 2:
                try:
                    x, y = map(float, parts)
                    coords.append((x, y))
                    lines.append(i)  # 원본 파일의 실제 라인 번호
                except ValueError:
                    continue
    return coords, lines


def enable_scroll_zoom(fig, ax, base_scale=1.25):
    """마우스 휠로 커서 기준 확대/축소"""
    def on_scroll(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        if event.button == 'up':
            scale = 1 / base_scale
        elif event.button == 'down':
            scale = base_scale
        else:
            return

        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        xdata, ydata = event.xdata, event.ydata

        new_w = (cur_xlim[1] - cur_xlim[0]) * scale
        new_h = (cur_ylim[1] - cur_ylim[0]) * scale

        relx = (xdata - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0])
        rely = (ydata - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0])

        ax.set_xlim([xdata - new_w * relx, xdata + new_w * (1 - relx)])
        ax.set_ylim([ydata - new_h * rely, ydata + new_h * (1 - rely)])

        ax.set_aspect('equal', adjustable='box')
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('scroll_event', on_scroll)


def enable_middle_drag_pan(fig, ax):
    """
    Middle-click(버튼=2) + 드래그로 뷰를 패닝하는 핸들러를 활성화한다.
    반환값: (press_id, release_id, motion_id) 연결 id 튜플
    """
    state = {"press_xy": None, "xlim0": None, "ylim0": None}

    def on_press(event):
        if event.inaxes != ax or event.button != 2:
            return
        if event.xdata is None or event.ydata is None:
            return
        state["press_xy"] = (event.xdata, event.ydata)
        state["xlim0"] = ax.get_xlim()
        state["ylim0"] = ax.get_ylim()

    def on_motion(event):
        if state["press_xy"] is None:
            return
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        x0, y0 = state["press_xy"]
        dx = x0 - event.xdata
        dy = y0 - event.ydata
        xlim0 = state["xlim0"]
        ylim0 = state["ylim0"]
        ax.set_xlim(xlim0[0] + dx, xlim0[1] + dx)
        ax.set_ylim(ylim0[0] + dy, ylim0[1] + dy)
        fig.canvas.draw_idle()

    def on_release(event):
        if event.button != 2:
            return
        state["press_xy"] = None
        state["xlim0"] = None
        state["ylim0"] = None

    cid_press = fig.canvas.mpl_connect('button_press_event', on_press)
    cid_motion = fig.canvas.mpl_connect('motion_notify_event', on_motion)
    cid_release = fig.canvas.mpl_connect('button_release_event', on_release)
    return (cid_press, cid_release, cid_motion)


def plot_multiple_rddf(file_paths, labels=None):
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.subplots_adjust(right=0.8)

    default_colors = ["red", "blue", "green", "orange", "purple", "black"]
    all_coords = []
    point_meta = []  # (file_path, line_no)
    n = len(file_paths)

    for idx, path_file in enumerate(file_paths):
        if not os.path.exists(path_file):
            continue
        coords, lines = parse_coords_and_lines(path_file)
        if not coords:
            continue

        label = labels[idx] if labels and idx < len(labels) else os.path.basename(path_file)
        color = default_colors[idx % len(default_colors)]
        plot_path(coords, ax, label=label, color=color,
                  plot_start=(n == 1), only_line=(n > 1))

        for (x, y), ln in zip(coords, lines):
            all_coords.append((x, y))
            point_meta.append((path_file, ln))

    click_markers = []
    circle_artist = None
    center_marker = None
    annot = ax.annotate("", xy=(0, 0), xytext=(15, 15), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    info_annot = ax.annotate("", xy=(0, 0), xytext=(10, -10), textcoords="offset points",
                             bbox=dict(boxstyle="round", fc="w"), fontsize=8)
    info_annot.set_visible(False)

    selected = []  # [(x, y, idx_min)]

    def reset_selection():
        nonlocal click_markers, circle_artist, center_marker
        for m in click_markers:
            m.remove()
        click_markers.clear()
        if circle_artist:
            circle_artist.remove()
            circle_artist = None
        if center_marker:
            center_marker.remove()
            center_marker = None
        annot.set_visible(False)

    def circumcircle(p1, p2, p3):
        (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
        # 삼각형 넓이 (두 벡터의 외적의 절반)
        area2 = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        if abs(area2) < 1e-12:
            return None, None, None  # 세 점이 일직선
        d1 = math.hypot(x2 - x1, y2 - y1)
        d2 = math.hypot(x3 - x2, y3 - y2)
        d3 = math.hypot(x1 - x3, y1 - y3)
        R = (d1 * d2 * d3) / (2.0 * abs(area2))  # 4*Area = 2*|area2|

        # 외심 (선형대수 방식)
        A = x1 * x1 + y1 * y1
        B = x2 * x2 + y2 * y2
        C = x3 * x3 + y3 * y3
        D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        if abs(D) < 1e-12:
            return None, None, None
        ux = (A * (y2 - y3) + B * (y3 - y1) + C * (y1 - y2)) / D
        uy = (A * (x3 - x2) + B * (x1 - x3) + C * (x2 - x1)) / D
        return (ux, uy), R, (d1, d2, d3)

    def on_click(event):
        if event.button != 1 or event.inaxes != ax:
            return
        if not all_coords:
            return
        if event.xdata is None or event.ydata is None:
            return
        nonlocal circle_artist, center_marker

        cx, cy = event.xdata, event.ydata
        dists = [math.hypot(x - cx, y - cy) for x, y in all_coords]
        idx_min = dists.index(min(dists))
        x_sel, y_sel = all_coords[idx_min]
        file_sel, line_sel = point_meta[idx_min]

        # 새 시퀀스 시작 조건: 3점을 이미 선택했으면 초기화
        if len(selected) == 3:
            reset_selection()
            selected.clear()

        # 점 표시
        m = ax.scatter(x_sel, y_sel, s=100, edgecolors='black', facecolors='none', linewidths=2)
        click_markers.append(m)
        selected.append(((x_sel, y_sel), idx_min))

        # 마지막 클릭점 간단 표기
        info_annot.xy = (x_sel, y_sel)
        info_annot.set_text(f"{os.path.basename(file_sel)} : line {line_sel}")
        info_annot.set_visible(True)

        # 3점이 모이면 외접원 계산
        if len(selected) == 3:
            p1, i1 = selected[0]
            p2, i2 = selected[1]
            p3, i3 = selected[2]
            center, R, sides = circumcircle(p1, p2, p3)
            if center is None or R is None:
                annot.xy = ((p1[0] + p2[0] + p3[0]) / 3, (p1[1] + p2[1] + p3[1]) / 3)
                annot.set_text("세 점이 일직선이어서 외접원이 없습니다")
                annot.set_visible(True)
            else:
                ux, uy = center
                # 원 그리기 및 외심 마커
                if circle_artist:
                    circle_artist.remove()
                circle_artist = patches.Circle((ux, uy), R, fill=False, linestyle='--', linewidth=1.5, color='black')
                ax.add_patch(circle_artist)

                if center_marker:
                    center_marker.remove()
                center_marker = ax.scatter(ux, uy, s=60, c='black', marker='+')

                # 주석: 반지름 및 선택된 포인트 정보
                annot.xy = (ux, uy)
                f1 = point_meta[i1]
                f2 = point_meta[i2]
                f3 = point_meta[i3]
                annot.set_text(
                    f"Circumcircle R = {R:.3f} m\n"
                    f"P1: {os.path.basename(f1[0])}:{f1[1]}\n"
                    f"P2: {os.path.basename(f2[0])}:{f2[1]}\n"
                    f"P3: {os.path.basename(f3[0])}:{f3[1]}"
                )
                annot.set_visible(True)

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', on_click)
    setup_plot(ax)
    enable_scroll_zoom(fig, ax)
    enable_middle_drag_pan(fig, ax)
    plt.show()


if __name__ == "__main__":
    files = [
        # "backward.txt",
        # "rddf_jeju2025_official.txt",
        # "2024-10-19_10-03_semi_1.txt",
        "steer7.txt",
        # "steer7_2.txt",
        # "steer-7.txt",
        "steer-7_2.txt"

    ]
    labels = [
        # "backward.txt",
        # "rddf_jeju2025_official.txt",
        # "2024-10-19_10-03_semi_1.txt",
        "steer7.txt",
        # "steer7_2.txt",
        # "steer-7.txt",
        "steer-7_2.txt"
    ]
    plot_multiple_rddf(files, labels)
