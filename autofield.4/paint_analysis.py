import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def analyze_line(image_name, real_avg_width_in=5):
    possible_extensions = ["", ".jpg", ".png", ".jpeg"]
    image_path = None
    for ext in possible_extensions:
        test_path = image_name + ext
        if os.path.exists(test_path):
            image_path = test_path
            break
    if image_path is None:
        print("Error: Image not found.")
        return

    img = cv2.imread(image_path)
    original = img.copy()
    h, w = img.shape[:2]

    # -------------------------
    # ISOLATE WHITE LINE using HSV
    # -------------------------
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,  0,  210]) #edit as needed
    upper_white = np.array([180, 35, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    white_mask[cv2.inRange(hsv, lower_green, upper_green) == 255] = 0

    lower_brown = np.array([10, 30, 60])
    upper_brown = np.array([30, 200, 200])
    white_mask[cv2.inRange(hsv, lower_brown, upper_brown) == 255] = 0

    kernel = np.ones((5, 5), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_mask = np.zeros_like(white_mask)
    for cnt in contours:
        if cv2.contourArea(cnt) > 2000:
            cv2.drawContours(filtered_mask, [cnt], -1, 255, thickness=-1)
    white_mask = filtered_mask

    # -------------------------
    # FIND EDGE POINTS per row
    # -------------------------
    left_points  = []
    right_points = []

    for y in range(h):
        row = white_mask[y]
        indices = np.where(row == 255)[0]
        if len(indices) >= 10:
            left_points.append((y, indices[0]))
            right_points.append((y, indices[-1]))

    if len(left_points) < 20 or len(right_points) < 20:
        print("Not enough edge points detected. Try lowering HSV brightness threshold.")
        return

    left_points  = np.array(left_points)
    right_points = np.array(right_points)

    # -------------------------
    # OUTLIER REJECTION
    # -------------------------
    def reject_outliers(points, iqr_factor=1.5):
        x_vals = points[:, 1]
        q1, q3 = np.percentile(x_vals, 25), np.percentile(x_vals, 75)
        iqr = q3 - q1
        mask = (x_vals >= q1 - iqr_factor * iqr) & (x_vals <= q3 + iqr_factor * iqr)
        return points[mask]

    left_points  = reject_outliers(left_points)
    right_points = reject_outliers(right_points)

    if len(left_points) < 20 or len(right_points) < 20:
        print("Too many outliers removed. Try raising iqr_factor.")
        return

    # -------------------------
    # FIT POLYNOMIAL TO EACH EDGE (weighted by row confidence)
    # -------------------------
    POLY_DEGREE = 2

    left_weights  = np.array([
        np.sum(white_mask[y] == 255) for y in left_points[:, 0]
    ], dtype=float)
    right_weights = np.array([
        np.sum(white_mask[y] == 255) for y in right_points[:, 0]
    ], dtype=float)

    left_weights  /= left_weights.max()
    right_weights /= right_weights.max()

    left_fit  = np.polyfit(left_points[:, 0],  left_points[:, 1],  POLY_DEGREE, w=left_weights)
    right_fit = np.polyfit(right_points[:, 0], right_points[:, 1], POLY_DEGREE, w=right_weights)

    left_poly  = np.poly1d(left_fit)
    right_poly = np.poly1d(right_fit)

    y_vals  = np.arange(h)
    left_x  = np.clip(left_poly(y_vals).astype(int),  0, w - 1)
    right_x = np.clip(right_poly(y_vals).astype(int), 0, w - 1)

    raw_widths = right_x - left_x
    median_w   = np.median(raw_widths[raw_widths > 0])
    valid_rows = np.abs(raw_widths - median_w) < (median_w * 0.5)

    # -------------------------
    # BUILD MASK FROM POLYNOMIAL CURVES
    # -------------------------
    poly_mask = np.zeros((h, w), dtype=np.uint8)
    for y in range(h):
        if valid_rows[y]:
            lx = left_x[y]
            rx = right_x[y]
            if rx > lx:
                poly_mask[y, lx:rx] = 255

    # -------------------------
    # OPACITY
    # -------------------------
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    OPACITY_THRESHOLD = 100 #edit as needed

    line_pixels_all = gray[poly_mask == 255]
    painted_pixels  = line_pixels_all[line_pixels_all >= OPACITY_THRESHOLD]

    coverage   = len(painted_pixels) / len(line_pixels_all) if len(line_pixels_all) > 0 else 0
    brightness = np.mean(painted_pixels) / 255 if len(painted_pixels) > 0 else 0
    opacity    = coverage * brightness

    # -------------------------
    # WIDTH per row (pixels → inches)
    # -------------------------
    widths_px    = raw_widths[valid_rows].astype(float)
    avg_width_px = np.mean(widths_px)
    std_width_px = np.std(widths_px)
    scale        = real_avg_width_in / avg_width_px
    std_width_in = std_width_px * scale

    # -------------------------
    # RESULTS
    # -------------------------
    print(f"Image: {image_path}")
    print(f"Coverage (fraction painted): {coverage:.3f}")
    print(f"Brightness (of painted px):  {brightness:.3f}")
    print(f"Opacity score (combined):    {opacity:.3f}")
    print(f"Avg Width: {avg_width_px:.1f} px = {real_avg_width_in:.2f} in (calibrated)")
    print(f"Width Std Dev: {std_width_px:.1f} px = {std_width_in:.3f} in")

    # -------------------------
    # VISUALIZE
    # -------------------------
    overlay = original.copy()
    overlay[poly_mask == 255] = [0, 255, 0]
    blended = cv2.addWeighted(original, 0.6, overlay, 0.4, 0)

    for y in range(h - 1):
        if valid_rows[y]:
            cv2.line(blended, (left_x[y],  y), (left_x[y+1],  y+1), (255, 0, 0), 2)
            cv2.line(blended, (right_x[y], y), (right_x[y+1], y+1), (0, 0, 255), 2)

    # -------------------------
    # AVG WIDTH MEASUREMENT INDICATOR
    # -------------------------
    # Pick a representative row near the vertical midpoint of valid rows
    valid_row_indices = np.where(valid_rows)[0]
    mid_idx = len(valid_row_indices) // 2
    meas_y  = int(valid_row_indices[mid_idx])
    lx_mid  = int(left_x[meas_y])
    rx_mid  = int(right_x[meas_y])
    avg_w_px_int = int(round(avg_width_px))

    # Center the indicator arrow on the midpoint x of the stripe
    cx       = (lx_mid + rx_mid) // 2
    arrow_lx = cx - avg_w_px_int // 2
    arrow_rx = cx + avg_w_px_int // 2

    ARROW_COLOR  = (0, 220, 255)   # bright cyan
    OUTLINE_COLOR = (0, 0, 0)      # black outline for contrast
    LINE_THICK   = max(3, int(w / 300))        # thicker shaft, scales with image
    TICK_LEN     = max(20, int(h / 25))        # taller ticks, scales with image
    ARROW_TIP    = max(15, int(w / 80))        # bigger arrowhead

    def draw_outlined_line(img, pt1, pt2, color, outline, thick):
        cv2.line(img, pt1, pt2, outline, thick + 4)
        cv2.line(img, pt1, pt2, color,   thick)

    def draw_outlined_arrowed(img, pt1, pt2, color, outline, thick, tip):
        cv2.arrowedLine(img, pt1, pt2, outline, thick + 4, tipLength=tip)
        cv2.arrowedLine(img, pt1, pt2, color,   thick,     tipLength=tip)

    # Horizontal shaft
    draw_outlined_line(blended,
                       (arrow_lx, meas_y), (arrow_rx, meas_y),
                       ARROW_COLOR, OUTLINE_COLOR, LINE_THICK)

    # Left arrowhead (pointing left)
    draw_outlined_arrowed(blended,
                          (arrow_lx + ARROW_TIP * 3, meas_y), (arrow_lx, meas_y),
                          ARROW_COLOR, OUTLINE_COLOR, LINE_THICK, 0.4)

    # Right arrowhead (pointing right)
    draw_outlined_arrowed(blended,
                          (arrow_rx - ARROW_TIP * 3, meas_y), (arrow_rx, meas_y),
                          ARROW_COLOR, OUTLINE_COLOR, LINE_THICK, 0.4)

    # Vertical end ticks
    draw_outlined_line(blended,
                       (arrow_lx, meas_y - TICK_LEN), (arrow_lx, meas_y + TICK_LEN),
                       ARROW_COLOR, OUTLINE_COLOR, LINE_THICK)
    draw_outlined_line(blended,
                       (arrow_rx, meas_y - TICK_LEN), (arrow_rx, meas_y + TICK_LEN),
                       ARROW_COLOR, OUTLINE_COLOR, LINE_THICK)

    # Label: two lines — inches on top, pixels below
    font         = cv2.FONT_HERSHEY_SIMPLEX
    font_scale   = max(1.2, w / 600)           # much larger font, scales with image
    thickness    = max(2, int(font_scale * 2))
    label_in     = f"{real_avg_width_in:.2f} in"
    label_px     = f"({avg_width_px:.0f} px)"

    (tw1, th1), bl1 = cv2.getTextSize(label_in, font, font_scale, thickness)
    (tw2, th2), bl2 = cv2.getTextSize(label_px, font, font_scale * 0.7, thickness - 1)

    line_gap   = 8
    box_w      = max(tw1, tw2) + 24
    box_h      = th1 + th2 + line_gap + 20
    text_x     = max(0, cx - box_w // 2)
    box_top    = max(0, meas_y - TICK_LEN - box_h - 10)

    # Rounded-rect background (simulated with filled rect + circles at corners)
    r = 10
    bx1, by1 = text_x, box_top
    bx2, by2 = text_x + box_w, box_top + box_h
    cv2.rectangle(blended, (bx1 + r, by1), (bx2 - r, by2), (20, 20, 20), cv2.FILLED)
    cv2.rectangle(blended, (bx1, by1 + r), (bx2, by2 - r), (20, 20, 20), cv2.FILLED)
    for cx_, cy_ in [(bx1+r, by1+r), (bx2-r, by1+r), (bx1+r, by2-r), (bx2-r, by2-r)]:
        cv2.circle(blended, (cx_, cy_), r, (20, 20, 20), cv2.FILLED)

    # Draw each text line with a black outline for extra crispness
    x1 = text_x + (box_w - tw1) // 2
    y1 = box_top + th1 + 10
    x2 = text_x + (box_w - tw2) // 2
    y2 = y1 + th2 + line_gap

    for dx, dy in [(-2,0),(2,0),(0,-2),(0,2)]:  # outline pass
        cv2.putText(blended, label_in, (x1+dx, y1+dy), font, font_scale,
                    OUTLINE_COLOR, thickness + 2, cv2.LINE_AA)
        cv2.putText(blended, label_px, (x2+dx, y2+dy), font, font_scale * 0.7,
                    OUTLINE_COLOR, thickness,     cv2.LINE_AA)

    cv2.putText(blended, label_in, (x1, y1), font, font_scale,
                ARROW_COLOR, thickness, cv2.LINE_AA)
    cv2.putText(blended, label_px, (x2, y2), font, font_scale * 0.7,
                (180, 220, 255), thickness - 1, cv2.LINE_AA)

    # Pure white = counted as painted, pure black = excluded
    opacity_vis = np.zeros_like(gray)
    opacity_vis[(poly_mask == 255) & (gray >= OPACITY_THRESHOLD)] = 255

    plt.figure(figsize=(20, 5))

    plt.subplot(1, 4, 1)
    plt.title("Original")
    plt.imshow(cv2.cvtColor(original, cv2.COLOR_BGR2RGB))

    plt.subplot(1, 4, 2)
    plt.title("HSV White Mask (filtered)")
    plt.imshow(white_mask, cmap='gray')

    plt.subplot(1, 4, 3)
    plt.title("Poly Edges + Fill + Avg Width Indicator")
    plt.imshow(cv2.cvtColor(blended, cv2.COLOR_BGR2RGB))

    plt.subplot(1, 4, 4)
    plt.title(f"Opacity Pixels (threshold={OPACITY_THRESHOLD})\nwhite=counted, black=excluded")
    plt.imshow(opacity_vis, cmap='gray')

    plt.tight_layout()
    plt.show()

analyze_line("circle", real_avg_width_in=5)
