import os
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


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


def plot_multiple_rddf(file_paths, labels=None):
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.subplots_adjust(right=0.8)

    default_colors = ["red", "blue", "green", "orange", "purple", "black"]
    all_coords = []
    point_meta = []  # ← (file_path, line_no) 를 all_coords 와 같은 인덱스로 보관
    n = len(file_paths)

    for idx, path_file in enumerate(file_paths):
        if not os.path.exists(path_file):   # 없으면 건너뜀
            continue
        coords, lines = parse_coords_and_lines(path_file)
        if not coords:
            continue

        # 플로팅
        label = labels[idx] if labels and idx < len(labels) else os.path.basename(path_file)
        color = default_colors[idx % len(default_colors)]
        plot_path(coords, ax, label=label, color=color,
                  plot_start=(n == 1), only_line=(n > 1))

        # 전역 좌표/메타 축적
        for (x, y), ln in zip(coords, lines):
            all_coords.append((x, y))
            point_meta.append((path_file, ln))

    click_markers = []
    line_artist = None
    annot = ax.annotate("", xy=(0, 0), xytext=(15, 15), textcoords="offset points",
                        bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    # 최근 클릭 포인트의 파일/라인을 바로 보여줄 작은 주석
    info_annot = ax.annotate("", xy=(0, 0), xytext=(10, -10), textcoords="offset points",
                             bbox=dict(boxstyle="round", fc="w"), fontsize=8)
    info_annot.set_visible(False)

    prev_point = None
    prev_idx = None  # ← 첫 번째 선택 점의 인덱스(메타 조회용)

    def on_click(event):
        if event.button != 1 or event.inaxes != ax:
            return
        if not all_coords:  # 데이터 없을 때 방어
            return
        nonlocal prev_point, prev_idx, line_artist

        if event.xdata is None or event.ydata is None:
            return
        cx, cy = event.xdata, event.ydata
        dists = [math.hypot(x - cx, y - cy) for x, y in all_coords]
        idx_min = dists.index(min(dists))
        x_sel, y_sel = all_coords[idx_min]
        file_sel, line_sel = point_meta[idx_min]

        if prev_point is None:
            # 첫 클릭
            for m in click_markers:
                m.remove()
            click_markers.clear()
            if line_artist:
                line_artist.remove()
                line_artist = None
            annot.set_visible(False)

            m1 = ax.scatter(x_sel, y_sel, s=100, edgecolors='black', facecolors='none', linewidths=2)
            click_markers.append(m1)

            # 선택점의 파일/라인 표시
            info_annot.xy = (x_sel, y_sel)
            info_annot.set_text(f"{os.path.basename(file_sel)} : line {line_sel}")
            info_annot.set_visible(True)

            prev_point = (x_sel, y_sel)
            prev_idx = idx_min
        else:
            # 두 번째 클릭
            x0, y0 = prev_point
            file0, line0 = point_meta[prev_idx]

            m2 = ax.scatter(x_sel, y_sel, s=100, edgecolors='black', facecolors='none', linewidths=2)
            click_markers.append(m2)

            if line_artist:
                line_artist.remove()
            line_artist, = ax.plot([x0, x_sel], [y0, y_sel], linestyle='-', linewidth=1, color='black')

            dist = math.hypot(x_sel - x0, y_sel - y0)
            mid_x, mid_y = (x0 + x_sel) / 2, (y0 + y_sel) / 2

            # --- 각도 계산 추가: 동=0°, 반시계+ / 북=0°, 시계+ ---
            dx = x_sel - x0
            dy = y_sel - y0
            if dist > 0:
                # 동쪽(+x) 기준, 반시계 방향 양(+): 0~360°
                theta_east_ccw = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
                # 북쪽(+y) 기준, 시계 방향 양(+): 0~360°
                bearing_north_cw = (90.0 - theta_east_ccw) % 360.0
                angle_text = (
                    f"East=0°, CCW+: {theta_east_ccw:.1f}°\n"
                    f"North=0°, CW+: {bearing_north_cw:.1f}°"
                )
            else:
                angle_text = "Angle: N/A (same point)"


            # 두 점의 파일/라인 + 거리 표시
            annot.xy = (mid_x, mid_y)
            annot.set_text(
                f"{os.path.basename(file0)}:{line0}  ↔  {os.path.basename(file_sel)}:{line_sel}\n"
                f"Distance: {dist:.2f} m\n"
                f"{angle_text}"
            )
            annot.set_visible(True)

            # 마지막 클릭점에 대한 짧은 정보표시는 최신 점으로 업데이트
            info_annot.xy = (x_sel, y_sel)
            info_annot.set_text(f"{os.path.basename(file_sel)} : line {line_sel}")
            info_annot.set_visible(True)

            prev_point = None
            prev_idx = None

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', on_click)
    setup_plot(ax)
    enable_scroll_zoom(fig, ax)
    enable_middle_drag_pan(fig, ax)
    plt.show()


def enable_scroll_zoom(fig, ax, base_scale=1.25):
    """마우스 휠로 커서 기준 확대/축소"""
    def on_scroll(event):
        # 축 밖이거나 좌표를 못 읽으면 무시
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return

        # 'up' = 줌 인, 'down' = 줌 아웃
        if event.button == 'up':
            scale = 1 / base_scale
        elif event.button == 'down':
            scale = base_scale
        else:
            return

        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        xdata, ydata = event.xdata, event.ydata

        # 새 가시 영역 크기
        new_w = (cur_xlim[1] - cur_xlim[0]) * scale
        new_h = (cur_ylim[1] - cur_ylim[0]) * scale

        # 커서 위치 비율 유지
        relx = (xdata - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0])
        rely = (ydata - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0])

        ax.set_xlim([xdata - new_w * relx, xdata + new_w * (1 - relx)])
        ax.set_ylim([ydata - new_h * rely, ydata + new_h * (1 - rely)])

        # 1:1 비율 유지(원래 코드가 equal이면 계속 유지)
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
        # 축 밖이거나 중클릭이 아니면 무시
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
        # 드래그 방향에 맞춰 창을 이동(패닝)
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

if __name__ == "__main__":
    files = [
        # "backward.txt",
        # "rddf_jeju2025_official.txt",
        # "2024-10-19_10-03_semi_1.txt",
        "semi_sim_to_real_1.txt",
        "2025-8-2_12-35_semi.txt",
        "semi_sim_to_real_2.txt"
    ]
    labels = [
        # "backward.txt",
        # "rddf_jeju2025_official.txt",
        # "2024-10-19_10-03_semi_1.txt",
        "semi_sim_to_real_1.txt",
        "2025-8-2_12-35_semi.txt",
        "semi_sim_to_real_2.txt"
    ]
    plot_multiple_rddf(files, labels)
