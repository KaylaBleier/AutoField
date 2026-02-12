import math

L = 100 
W = 70

def draw_soccer_pitch(L=100.0, W=70.0, cols=100, rows=35):
    grid = [[' ' for _ in range(cols)] for _ in range(rows)]
    
    def to_grid(x, y):
        # x, y are in field units (0–L, 0–W)
        c = int((x / L) * (cols - 1))
        r = rows - 1 - int((y / W) * (rows - 1))
        return max(0, min(rows - 1, r)), max(0, min(cols - 1, c))
    
    def draw_line(x1, y1, x2, y2, ch='-'):
        steps = max(abs(x2 - x1), abs(y2 - y1)) * 4
        for i in range(int(steps) + 1):
            t = i / max(1, steps)
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            r, c = to_grid(x, y)
            grid[r][c] = ch
    
    def draw_arc(cx, cy, radius, start_angle, end_angle, ch='o'):
        steps = 16
        for i in range(steps + 1):
            angle = start_angle + (end_angle - start_angle) * i / steps
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            r, c = to_grid(x, y)
            grid[r][c] = ch
    
    # FIELD BOUNDARY
    draw_line(0, 0, L, 0, '-')      # bottom
    draw_line(L, 0, L, W, '|')      # right  
    draw_line(L, W, 0, W, '-')      # top
    draw_line(0, W, 0, 0, '|')      # left
    
    # CENTER LINE
    draw_line(L / 2, 0, L / 2, W, '|')
    
    # CENTER CIRCLE
    center_radius = 0.15 * W
    draw_arc(L / 2, W / 2, center_radius, 0, 2 * math.pi, 'o')
    
    # PENALTY AREAS (16.5m × 40.3m)
    penalty_depth = 0.15 * L
    penalty_width = 0.59 * W
    py0 = (W - penalty_width) / 2
    py1 = (W + penalty_width) / 2
    
    # LEFT PENALTY BOX
    draw_line(0, py0, penalty_depth, py0, '-')
    draw_line(penalty_depth, py0, penalty_depth, py1, '|')
    draw_line(penalty_depth, py1, 0, py1, '-')
    draw_line(0, py1, 0, py0, '|')

    # RIGHT PENALTY BOX  
    draw_line(L, py0, L - penalty_depth, py0, '-')
    draw_line(L - penalty_depth, py0, L - penalty_depth, py1, '|')
    draw_line(L - penalty_depth, py1, L, py1, '-')
    draw_line(L, py1, L, py0, '|')
    
    # GOAL AREAS
    goal_depth = 0.05 * L
    goal_width = 0.27 * W
    gy0 = (W - goal_width) / 2
    gy1 = (W + goal_width) / 2
    
    # LEFT GOAL AREA
    draw_line(0, gy0, goal_depth, gy0, '_')
    draw_line(goal_depth, gy0, goal_depth, gy1, '|')
    draw_line(goal_depth, gy1, 0, gy1, '_')
    draw_line(0, gy1, 0, gy0, '|')
    
    # RIGHT GOAL AREA
    draw_line(L, gy0, L - goal_depth, gy0, '_')
    draw_line(L - goal_depth, gy0, L - goal_depth, gy1, '|')
    draw_line(L - goal_depth, gy1, L, gy1, '_')
    draw_line(L, gy1, L, gy0, '|')
    
    # PENALTY MARKS
    penalty_mark_dist = 0.10 * L
    r, c = to_grid(penalty_mark_dist, W / 2)
    grid[r][c] = '*'
    r, c = to_grid(L - penalty_mark_dist, W / 2)
    grid[r][c] = '*'
    
    # CORNER ARCS (1m radius)
    draw_arc(1, 1, 1, math.pi, 3 * math.pi / 2, '+')
    draw_arc(L - 1, 1, 1, math.pi / 2, math.pi, '+')
    draw_arc(L - 1, W - 1, 1, 0, math.pi / 2, '+')
    draw_arc(1, W - 1, 1, 3 * math.pi / 2, 2 * math.pi, '+')
    
    # PRINT
    print('+' + '-' * cols + '+')
    for row in grid:
        print('|' + ''.join(row) + '|')
    print('+' + '-' * cols + '+')

def generate_complete_waypoints(L=100.0, W=70.0):
    penalty_depth = 0.15 * L
    penalty_width = 0.59 * W
    py0 = (W - penalty_width) / 2
    py1 = (W + penalty_width) / 2
    
    goal_depth = 0.05 * L
    goal_width = 0.27 * W
    gy0 = (W - goal_width) / 2
    gy1 = (W + goal_width) / 2
    
    waypoints = []
    
    # 1. OUTER BOUNDARY
    waypoints.extend([
        (0, 0), (L, 0), (L, W), (0, W), (0, 0)
    ])
    
    # 2. CENTER LINE
    waypoints.extend([(L / 2, 0), (L / 2, W)])
    
    # 3. LEFT PENALTY BOX
    waypoints.extend([
        (0, py0), (penalty_depth, py0),
        (penalty_depth, py1), (0, py1), (0, py0)
    ])
    
    # 4. RIGHT PENALTY BOX
    waypoints.extend([
        (L, py0), (L - penalty_depth, py0),
        (L - penalty_depth, py1), (L, py1), (L, py0)
    ])
    
    # 5. LEFT GOAL BOX
    waypoints.extend([
        (0, gy0), (goal_depth, gy0),
        (goal_depth, gy1), (0, gy1), (0, gy0)
    ])
    
    # 6. RIGHT GOAL BOX
    waypoints.extend([
        (L, gy0), (L - goal_depth, gy0),
        (L - goal_depth, gy1), (L, gy1), (L, gy0)
    ])
    
    # 7. PENALTY MARKS (scaled with L)
    penalty_mark_dist = 0.10 * L
    waypoints.extend([
        (penalty_mark_dist, W / 2),
        (L - penalty_mark_dist, W / 2)
    ])
    
    return waypoints

# RUN
if __name__ == "__main__":
    print("FIELD VISUALIZATION")
    draw_soccer_pitch(L, W)
    
    print("\nWAYPOINTS FOR RTK-GPS")
    wpts = generate_complete_waypoints(L, W)
    print(f"Total: {len(wpts)} waypoints")
    for i, (x, y) in enumerate(wpts):
        print(f"{i:2d}: ({x:5.1f}, {y:5.1f})")
