import pygame
import math
import heapq
from collections import deque

pygame.init()

# ============================================================
# INDUSTRIAL AMR - TWO ROBOT AUTONOMOUS MATERIAL HANDLING
# AMR-01: P1-P6 -> loading zone
# AMR-02: starts moving stock from upper unloading zone to six racks
#         at the same time AMR-01 starts its mission.
# ============================================================

WIDTH, HEIGHT = 1280, 800
CELL = 20
FPS = 60
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Industrial AMR - Autonomous Two-Robot Warehouse")
clock = pygame.time.Clock()

# Colours
FLOOR = (174, 181, 181)
FLOOR_TILE = (164, 172, 173)
GRID = (145, 153, 154)
WHITE = (245, 245, 245)
BLACK = (25, 28, 30)
DARK = (55, 62, 66)
MID = (105, 115, 120)
LIGHT = (205, 210, 210)
BLUE = (35, 105, 165)
DARK_BLUE = (20, 55, 90)
STEEL = (75, 100, 115)
BROWN = (145, 95, 50)
WOOD = (175, 125, 75)
YELLOW = (255, 195, 0)
ORANGE = (245, 125, 25)
GREEN = (30, 190, 85)
RED = (220, 50, 50)
TRAIL_BLUE = (20, 90, 255)

font = pygame.font.SysFont("arial", 20)
small = pygame.font.SysFont("arial", 16)
tiny = pygame.font.SysFont("arial", 13)
title_font = pygame.font.SysFont("arial", 22, bold=True)

# Robot settings
ROBOT_RADIUS = 18
SAFE_DISTANCE = 58
AMR1_SPEED = 145.0
AMR2_SPEED = 135.0
ROTATION_SPEED = 180.0

# Static factory equipment
obstacles = [
    {"rect": pygame.Rect(180, 130, 120, 180), "kind": "rack"},
    {"rect": pygame.Rect(430, 130, 120, 180), "kind": "rack"},
    {"rect": pygame.Rect(680, 130, 120, 180), "kind": "rack"},
    {"rect": pygame.Rect(180, 510, 120, 180), "kind": "rack"},
    {"rect": pygame.Rect(430, 510, 120, 180), "kind": "rack"},
    {"rect": pygame.Rect(680, 510, 120, 180), "kind": "rack"},
    {"rect": pygame.Rect(930, 300, 130, 100), "kind": "machine"},
]
dynamic_obstacles = []
moving_obstacles = [
    # Keep the forklift as the only moving factory obstacle.
    {"rect": pygame.Rect(340, 385, 50, 28), "kind": "forklift", "vx": 72.0, "vy": 0.0,
     "min_x": 320, "max_x": 850, "min_y": 385, "max_y": 385},
]

# Pickup points are placed in clear aisles beside the six racks.
# Each coordinate is reachable by the grid planner without entering a rack.
PICKUPS = [
    (350.0, 340.0), (600.0, 340.0), (850.0, 340.0),
    (350.0, 480.0), (600.0, 480.0), (850.0, 480.0),
]
LOADING_ZONE = pygame.Rect(1100, 610, 150, 150)
DELIVERY_STATION = (1080.0, 635.0)  # robot approach point outside the loading-zone slots
LOADING_SLOTS = [
    (1125.0, 675.0), (1175.0, 675.0), (1225.0, 675.0),
    (1125.0, 715.0), (1175.0, 715.0), (1225.0, 715.0),
]
# Shared charging station with two separate docking bays.
CHARGING_STATION = pygame.Rect(820, 685, 180, 95)
AMR1_CHARGE_POINT = (865.0, 735.0)
AMR2_CHARGE_POINT = (945.0, 735.0)
CHARGE_DURATION = 5.0
BATTERY_DRAIN_PER_PIXEL = 0.008
# AMR-02 takes six fresh boxes from the upper-right unloading zone.
UNLOADING_ZONE = pygame.Rect(930, 95, 300, 185)
UNLOADING_SLOTS = [
    (975.0, 145.0), (1065.0, 145.0), (1155.0, 145.0),
    (975.0, 225.0), (1065.0, 225.0), (1155.0, 225.0),
]
# AMR-02 approaches from the clear aisle ABOVE each rack.
# These approach coordinates are deliberately outside the A* inflated rack zones.
# The box is then placed visually on the upper edge of the corresponding rack.
RACK_APPROACH_POINTS = [
    (240.0, 90.0), (490.0, 90.0), (740.0, 90.0),
    (240.0, 470.0), (490.0, 470.0), (740.0, 470.0),
]
RACK_DROP_POINTS = [
    (240.0, 128.0), (490.0, 128.0), (740.0, 128.0),
    (240.0, 508.0), (490.0, 508.0), (740.0, 508.0),
]

# Robot state
amr1 = {"x": 80.0, "y": 80.0, "angle": 0.0, "path": [], "goal": None,
        "status": "Waiting for task", "distance": 0.0, "trail": [(80, 80)],
        "last_trail": (80.0, 80.0), "carrying": None, "battery": 100.0}
amr2 = {"x": 80.0, "y": 720.0, "angle": 0.0, "path": [], "goal": None,
        "status": "Standby - waiting for mission start", "distance": 0.0, "trail": [(80, 720)],
        "last_trail": (80.0, 720.0), "carrying": None, "battery": 100.0}

# Package state: at_pickup -> on AMR-01 -> in_loading -> on AMR-02 -> returned
packages = [{"state": "at_pickup", "slot": None} for _ in PICKUPS]
amr2_stock = [{"state": "in_unloading"} for _ in UNLOADING_SLOTS]

# AMR-01 task state
amr1_task = "IDLE"
amr1_index = None
amr1_timer = 0.0
amr1_payload_progress = 0.0
amr1_completed = 0
amr1_charge_timer = 0.0

# AMR-02 task state
amr2_started = False
amr2_queue = deque()
amr2_task = "STANDBY"
amr2_index = None
amr2_timer = 0.0
amr2_payload_progress = 0.0
amr2_charge_timer = 0.0

# UI / navigation
show_info = False
show_controls = False
info_button = pygame.Rect(850, 10, 44, 34)
controls_button = pygame.Rect(910, 10, 44, 34)
info_panel = pygame.Rect(420, 10, 400, 350)
controls_panel = pygame.Rect(990, 10, 280, 355)
# Centralized fleet-management dashboard (toggle with F or the FLEET button).
fleet_button = pygame.Rect(970, 10, 88, 34)
fleet_panel = pygame.Rect(760, 52, 510, 570)
fleet_start_button = pygame.Rect(780, 570, 205, 38)
fleet_reset_button = pygame.Rect(995, 570, 255, 38)
show_fleet = False
manual_goal = None
manual_path = []
robot1_manual = False
sensor_range = 170
sensor_angles = [-40, 0, 40]
sensor_cooldown = 0.0

# ------------------------- geometry / pathfinding -------------------------
def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def robot_pos(robot):
    return (robot["x"], robot["y"])


def all_obstacle_rects():
    return ([o["rect"] for o in obstacles] + dynamic_obstacles +
            [o["rect"] for o in moving_obstacles])


def cell_of(x, y):
    return (int(x // CELL), int(y // CELL))


def cell_center(cell):
    return (cell[0] * CELL + CELL / 2, cell[1] * CELL + CELL / 2)

COLS, ROWS = WIDTH // CELL, HEIGHT // CELL


def build_blocked_grid(extra_points=None):
    grid = [[False for _ in range(ROWS)] for _ in range(COLS)]
    for rect in all_obstacle_rects():
        expanded = rect.inflate(ROBOT_RADIUS * 2 + 4, ROBOT_RADIUS * 2 + 4)
        left = max(0, expanded.left // CELL)
        right = min(COLS - 1, expanded.right // CELL)
        top = max(0, expanded.top // CELL)
        bottom = min(ROWS - 1, expanded.bottom // CELL)
        for gx in range(left, right + 1):
            for gy in range(top, bottom + 1):
                cx, cy = cell_center((gx, gy))
                if expanded.collidepoint(cx, cy):
                    grid[gx][gy] = True
    # Reserve a small buffer around the other robot's planned route.
    if extra_points:
        for point in extra_points:
            gx, gy = cell_of(*point)
            for ox in (-1, 0, 1):
                for oy in (-1, 0, 1):
                    nx, ny = gx + ox, gy + oy
                    if 0 <= nx < COLS and 0 <= ny < ROWS:
                        grid[nx][ny] = True
    return grid


def blocked(cell, grid):
    x, y = cell
    return x < 0 or y < 0 or x >= COLS or y >= ROWS or grid[x][y]


def astar(start_xy, goal_xy, grid):
    start, goal = cell_of(*start_xy), cell_of(*goal_xy)
    if start == goal:
        return [goal_xy]
    # Permit the robot's current cell even if its edge touches an inflated obstacle.
    grid[start[0]][start[1]] = False
    if blocked(goal, grid):
        return None
    open_heap = [(0, start)]
    came_from = {}
    g = {start: 0}
    visited = set()
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            cells = []
            while current != start:
                cells.append(current)
                current = came_from[current]
            cells.reverse()
            points = [cell_center(c) for c in cells]
            if points and distance(points[-1], goal_xy) > 2:
                points.append(goal_xy)
            return points or [goal_xy]
        for nxt in ((current[0]+1,current[1]), (current[0]-1,current[1]),
                    (current[0],current[1]+1), (current[0],current[1]-1)):
            if blocked(nxt, grid) or nxt in visited:
                continue
            ng = g[current] + 1
            if ng < g.get(nxt, 10**9):
                g[nxt] = ng
                came_from[nxt] = current
                h = abs(nxt[0]-goal[0]) + abs(nxt[1]-goal[1])
                heapq.heappush(open_heap, (ng+h, nxt))
    return None


def point_is_clear(x, y):
    if x < ROBOT_RADIUS or y < ROBOT_RADIUS or x > WIDTH-ROBOT_RADIUS or y > HEIGHT-ROBOT_RADIUS:
        return False
    for rect in all_obstacle_rects():
        nearest_x = max(rect.left, min(x, rect.right))
        nearest_y = max(rect.top, min(y, rect.bottom))
        if math.hypot(x-nearest_x, y-nearest_y) < ROBOT_RADIUS + 2:
            return False
    return True


def set_robot_goal(robot, goal, other=None, reserve_other_path=False):
    extra = []
    if other is not None:
        # AMR-02 yields to AMR-01's current route; AMR-01 has priority.
        if reserve_other_path:
            extra.extend(other["path"][:])
        extra.append(robot_pos(other))
    grid = build_blocked_grid(extra)
    route = astar(robot_pos(robot), goal, grid)
    if route is None:
        robot["path"] = []
        robot["goal"] = goal
        robot["status"] = "Waiting - route temporarily blocked"
        return False
    robot["path"] = route
    robot["goal"] = goal
    robot["status"] = "Autonomous navigation"
    return True


def replan_robot(robot, other=None, reserve_other_path=False):
    if robot["goal"] is not None:
        return set_robot_goal(robot, robot["goal"], other, reserve_other_path)
    return False


def collision_at(x, y):
    if x < ROBOT_RADIUS or y < ROBOT_RADIUS or x > WIDTH-ROBOT_RADIUS or y > HEIGHT-ROBOT_RADIUS:
        return True
    for rect in all_obstacle_rects():
        nearest_x = max(rect.left, min(x, rect.right))
        nearest_y = max(rect.top, min(y, rect.bottom))
        if math.hypot(x-nearest_x, y-nearest_y) < ROBOT_RADIUS:
            return True
    return False


def movement_safe(robot, nx, ny, other):
    current_d = distance(robot_pos(robot), robot_pos(other))
    next_d = distance((nx, ny), robot_pos(other))
    if next_d >= SAFE_DISTANCE:
        return True
    # If already in the safety buffer, only permit motion directly away.
    return current_d < SAFE_DISTANCE and next_d > current_d + 0.2


def route_conflicts(route_a, route_b, threshold=SAFE_DISTANCE):
    if not route_a or not route_b:
        return False
    # Check nearby future waypoints, not just exact grid-cell equality.
    for pa in route_a[:18]:
        for pb in route_b[:18]:
            if distance(pa, pb) < threshold:
                return True
    return False

# ------------------------- sensors / moving hazards -------------------------
def update_moving_obstacles(dt):
    for item in moving_obstacles:
        r = item["rect"]
        # Keep fractional movement between frames for smooth forklift motion.
        item["move_x"] = item.get("move_x", 0.0) + item["vx"] * dt
        item["move_y"] = item.get("move_y", 0.0) + item["vy"] * dt
        step_x = int(item["move_x"])
        step_y = int(item["move_y"])
        item["move_x"] -= step_x
        item["move_y"] -= step_y
        r.x += step_x
        r.y += step_y
        if r.left <= item["min_x"]:
            r.left = item["min_x"]
            item["vx"] = abs(item["vx"])
            item["move_x"] = 0.0
        elif r.right >= item["max_x"]:
            r.right = item["max_x"]
            item["vx"] = -abs(item["vx"])
            item["move_x"] = 0.0
        if r.top <= item["min_y"]:
            r.top = item["min_y"]
            item["vy"] = abs(item["vy"])
            item["move_y"] = 0.0
        elif r.bottom >= item["max_y"]:
            r.bottom = item["max_y"]
            item["vy"] = -abs(item["vy"])
            item["move_y"] = 0.0


def sensor_hit(robot, angle_deg):
    angle = math.radians(angle_deg)
    sx = robot["x"] + math.cos(angle) * ROBOT_RADIUS
    sy = robot["y"] - math.sin(angle) * ROBOT_RADIUS
    for d in range(0, sensor_range + 1, 4):
        x = sx + math.cos(angle) * d
        y = sy - math.sin(angle) * d
        if x < 0 or y < 0 or x >= WIDTH or y >= HEIGHT:
            return (x, y), d, True
        for rect in all_obstacle_rects():
            if rect.collidepoint(int(x), int(y)):
                return (x, y), d, True
    return (sx + math.cos(angle)*sensor_range, sy - math.sin(angle)*sensor_range), sensor_range, False

# ------------------------- task management -------------------------
def start_amr1_sequence():
    global amr1_task, amr1_index, amr1_completed
    if amr1_task not in ("IDLE", "CANCELLED", "COMPLETE"):
        return
    amr1_index = 0
    amr1_completed = 0
    for p in packages:
        p["state"] = "at_pickup"
        p["slot"] = None
    for item in amr2_stock:
        item["state"] = "in_unloading"
    amr1["carrying"] = None
    amr1["battery"] = 100.0
    amr1_task = "TO_PICKUP"
    set_robot_goal(amr1, PICKUPS[0], None, False)
    amr1["status"] = "Travelling to P1"
    # Start AMR-02 at the same time as AMR-01, not at the fourth pickup.
    start_amr2_return_service()


def start_amr1_next_pickup():
    global amr1_task, amr1_index, amr2_started, amr2_task
    if amr1_index is None:
        return
    if amr1_index >= len(PICKUPS):
        amr1_task = "COMPLETE"
        amr1["status"] = "All six packages delivered"
        return
    amr1_task = "TO_PICKUP"
    set_robot_goal(amr1, PICKUPS[amr1_index], amr2 if amr2_started else None, False)
    amr1["status"] = f"Travelling to P{amr1_index + 1}"


def start_amr2_return_service():
    """Start AMR-02's independent unloading-zone-to-racks mission."""
    global amr2_started, amr2_task
    amr2_started = True
    amr2["battery"] = 100.0
    amr2_queue.clear()
    for i, item in enumerate(amr2_stock):
        if item["state"] == "in_unloading":
            amr2_queue.append(i)
    if amr2_queue:
        amr2_task = "TO_UNLOADING_PACKAGE"
        amr2["status"] = "Collecting stock from upper unloading zone"
        begin_amr2_next_package()
    else:
        amr2_task = "WAITING_FOR_PACKAGE"
        amr2["status"] = "Waiting for stock in unloading zone"


def begin_amr2_next_package():
    global amr2_task, amr2_index
    if amr2_queue:
        amr2_index = amr2_queue.popleft()
        item = amr2_stock[amr2_index]
        if item["state"] != "in_unloading":
            begin_amr2_next_package()
            return
        ok = set_robot_goal(amr2, UNLOADING_SLOTS[amr2_index], amr1, True)
        if not ok:
            amr2_task = "WAITING_FOR_ROUTE"
        else:
            amr2_task = "TO_UNLOADING_PACKAGE"
            amr2["status"] = f"Collecting unloading box {amr2_index + 1}/6"
    else:
        amr2_index = None
        amr2_task = "WAITING_FOR_PACKAGE"
        amr2["status"] = "Waiting for next unloading-zone box"


def update_amr1_task(dt):
    global amr1_task, amr1_timer, amr1_payload_progress, amr1_completed, amr1_index
    global amr1_charge_timer
    if amr1_task == "TO_CHARGER":
        if not amr1["path"] and distance(robot_pos(amr1), AMR1_CHARGE_POINT) < 14:
            amr1_task = "CHARGING"
            amr1_charge_timer = CHARGE_DURATION
            amr1["status"] = "Charging battery for 5 seconds"
        return
    if amr1_task == "CHARGING":
        amr1_charge_timer = max(0.0, amr1_charge_timer - dt)
        if amr1_charge_timer <= 0.0:
            amr1["battery"] = 100.0
            amr1["status"] = "Battery fully charged - resuming mission"
            start_amr1_next_pickup()
        return
    if amr1_task == "TO_PICKUP":
        if not amr1["path"] and amr1_index is not None and distance(robot_pos(amr1), PICKUPS[amr1_index]) < 12:
            amr1_task = "PICKING"
            amr1_timer = 1.5
            amr1_payload_progress = 0.0
            amr1["status"] = f"Picking package P{amr1_index + 1}"
    elif amr1_task == "PICKING":
        amr1_timer -= dt
        amr1_payload_progress = min(1.0, amr1_payload_progress + dt / 1.5)
        if amr1_timer <= 0:
            packages[amr1_index]["state"] = "on_amr1"
            amr1["carrying"] = amr1_index
            amr1_task = "TO_LOADING"
            set_robot_goal(amr1, DELIVERY_STATION, amr2 if amr2_started else None, False)
            amr1["status"] = "Carrying package to loading zone"
    elif amr1_task == "TO_LOADING":
        if not amr1["path"] and distance(robot_pos(amr1), DELIVERY_STATION) < 14:
            amr1_task = "UNLOADING"
            amr1_timer = 1.5
            amr1_payload_progress = 1.0
            amr1["status"] = "Placing package in loading zone"
    elif amr1_task == "UNLOADING":
        amr1_timer -= dt
        amr1_payload_progress = max(0.0, amr1_payload_progress - dt / 1.5)
        if amr1_timer <= 0:
            idx = amr1["carrying"]
            packages[idx]["state"] = "in_loading"
            packages[idx]["slot"] = LOADING_SLOTS[idx]
            amr1["carrying"] = None
            amr1_completed += 1
            amr1_index += 1
            if amr1_completed == 4:
                amr1_task = "TO_CHARGER"
                if set_robot_goal(amr1, AMR1_CHARGE_POINT, amr2 if amr2_started else None, False):
                    amr1["status"] = "Fourth box delivered - travelling to charging station"
                else:
                    amr1["status"] = "Waiting for a clear route to charging station"
            else:
                start_amr1_next_pickup()


def update_amr2_task(dt):
    global amr2_task, amr2_timer, amr2_payload_progress, amr2_index
    global amr2_charge_timer
    if not amr2_started:
        return
    if amr2_task == "TO_CHARGER":
        if not amr2["path"] and distance(robot_pos(amr2), AMR2_CHARGE_POINT) < 14:
            amr2_task = "CHARGING"
            amr2_charge_timer = CHARGE_DURATION
            amr2["status"] = "Charging battery for 5 seconds"
        return
    if amr2_task == "CHARGING":
        amr2_charge_timer = max(0.0, amr2_charge_timer - dt)
        if amr2_charge_timer <= 0.0:
            amr2["battery"] = 100.0
            amr2["status"] = "Battery fully charged - resuming rack deliveries"
            begin_amr2_next_package()
        return
    if amr2_task == "WAITING_FOR_PACKAGE":
        for i, item in enumerate(amr2_stock):
            if item["state"] == "in_unloading" and i != amr2_index and i not in amr2_queue:
                amr2_queue.append(i)
        if amr2_queue:
            begin_amr2_next_package()
        return
    if amr2_task == "WAITING_FOR_CHARGER":
        if set_robot_goal(amr2, AMR2_CHARGE_POINT, amr1, True):
            amr2_task = "TO_CHARGER"
            amr2["status"] = "Travelling to charging station"
        return
    if amr2_task == "WAITING_FOR_ROUTE":
        if amr2_index is not None:
            if amr2_stock[amr2_index]["state"] == "on_amr2":
                retry_goal = RACK_APPROACH_POINTS[amr2_index]
            else:
                retry_goal = UNLOADING_SLOTS[amr2_index]
            if not amr1["path"] or not route_conflicts(amr1["path"], [robot_pos(amr2)]):
                if set_robot_goal(amr2, retry_goal, amr1, True):
                    amr2_task = "TO_RACK" if amr2_stock[amr2_index]["state"] == "on_amr2" else "TO_UNLOADING_PACKAGE"
        return
    if amr2_task == "TO_UNLOADING_PACKAGE":
        if not amr2["path"] and amr2_index is not None and distance(robot_pos(amr2), UNLOADING_SLOTS[amr2_index]) < 14:
            amr2_task = "PICKING_RETURN"
            amr2_timer = 1.2
            amr2_payload_progress = 0.0
            amr2["status"] = f"Picking box {amr2_index + 1} from unloading zone"
    elif amr2_task == "PICKING_RETURN":
        amr2_timer -= dt
        amr2_payload_progress = min(1.0, amr2_payload_progress + dt / 1.2)
        if amr2_timer <= 0:
            amr2_stock[amr2_index]["state"] = "on_amr2"
            amr2["carrying"] = amr2_index
            amr2_task = "TO_RACK"
            set_robot_goal(amr2, RACK_APPROACH_POINTS[amr2_index], amr1, True)
            amr2["status"] = f"Delivering box {amr2_index + 1} to rack {amr2_index + 1}"
    elif amr2_task == "TO_RACK":
        if not amr2["path"] and amr2_index is not None and distance(robot_pos(amr2), RACK_APPROACH_POINTS[amr2_index]) < 14:
            amr2_task = "UNLOADING_RETURN"
            amr2_timer = 1.2
            amr2_payload_progress = 1.0
            amr2["status"] = f"Placing box on rack {amr2_index + 1}"
    elif amr2_task == "UNLOADING_RETURN":
        amr2_timer -= dt
        amr2_payload_progress = max(0.0, amr2_payload_progress - dt / 1.2)
        if amr2_timer <= 0:
            amr2_stock[amr2_index]["state"] = "delivered"
            amr2["carrying"] = None
            amr2["status"] = f"Box {amr2_index + 1} delivered to rack"
            delivered_count = sum(item["state"] == "delivered" for item in amr2_stock)
            if delivered_count == 3:
                amr2_task = "TO_CHARGER"
                if set_robot_goal(amr2, AMR2_CHARGE_POINT, amr1, True):
                    amr2["status"] = "Third box delivered - travelling to charging station"
                else:
                    amr2_task = "WAITING_FOR_CHARGER"
                    amr2["status"] = "Waiting for a clear route to charging station"
            else:
                begin_amr2_next_package()

# ------------------------- movement / replanning -------------------------
def move_robot(robot, other, dt, is_amr2=False):
    global amr2_task
    if not robot["path"]:
        return
    # Replan before reaching a moving person/forklift if it blocks the next route segment.
    target = robot["path"][0]
    segment_distance = distance(robot_pos(robot), target)
    check_distance = min(segment_distance, 55.0)
    if check_distance > 4:
        for moving in moving_obstacles:
            hazard = moving["rect"].inflate(ROBOT_RADIUS * 2 + 12, ROBOT_RADIUS * 2 + 12)
            samples = max(2, int(check_distance // 8))
            if any(hazard.collidepoint(
                robot["x"] + (target[0] - robot["x"]) * k / samples,
                robot["y"] + (target[1] - robot["y"]) * k / samples
            ) for k in range(1, samples + 1)):
                replan_robot(robot, other, is_amr2)
                robot["status"] = f"{moving['kind'].title()} detected - replanning"
                return
    dx, dy = target[0]-robot["x"], target[1]-robot["y"]
    dist = math.hypot(dx, dy)
    if dist < 2.5:
        robot["x"], robot["y"] = target
        robot["path"].pop(0)
        return
    angle = math.degrees(math.atan2(-dy, dx))
    robot["angle"] = angle
    step = min((AMR2_SPEED if is_amr2 else AMR1_SPEED) * dt, dist)
    nx = robot["x"] + dx / dist * step
    ny = robot["y"] + dy / dist * step
    if collision_at(nx, ny):
        ok = replan_robot(robot, other, is_amr2)
        if is_amr2 and not ok and amr2_started:
            amr2_task = "WAITING_FOR_ROUTE"
        robot["status"] = "Obstacle detected - replanning route"
        return
    if is_amr2:
        # AMR-02 yields if its proposed route intersects AMR-01's near-future route.
        if route_conflicts([(nx, ny)] + robot["path"][:8], [robot_pos(amr1)] + amr1["path"][:8]):
            robot["status"] = "Yielding - AMR-01 has priority"
            ok = replan_robot(robot, other, True)
            if not ok:
                # The task manager retries this route when AMR-01 has moved clear.
                if amr2_started:
                    amr2_task = "WAITING_FOR_ROUTE"
            return
    if not movement_safe(robot, nx, ny, other):
        robot["status"] = "Safety stop - replanning around other AMR"
        ok = replan_robot(robot, other, is_amr2)
        if is_amr2 and not ok and amr2_started:
            amr2_task = "WAITING_FOR_ROUTE"
        return
    travelled = math.hypot(nx-robot["x"], ny-robot["y"])
    robot["distance"] += travelled
    robot["battery"] = max(5.0, robot.get("battery", 100.0) - travelled * BATTERY_DRAIN_PER_PIXEL)
    robot["x"], robot["y"] = nx, ny


def update_trail(robot):
    p = robot_pos(robot)
    if distance(p, robot["last_trail"]) >= 6:
        robot["trail"].append(p)
        robot["last_trail"] = p
        if len(robot["trail"]) > 500:
            robot["trail"].pop(0)

# ------------------------- drawing -------------------------
def draw_floor():
    screen.fill(FLOOR)
    for x in range(0, WIDTH, 80):
        pygame.draw.line(screen, FLOOR_TILE, (x, 0), (x, HEIGHT), 1)
    for y in range(0, HEIGHT, 80):
        pygame.draw.line(screen, FLOOR_TILE, (0, y), (WIDTH, y), 1)
    pygame.draw.rect(screen, (195, 183, 115), (0, 350, WIDTH, 110))
    pygame.draw.line(screen, YELLOW, (0, 350), (WIDTH, 350), 4)
    pygame.draw.line(screen, YELLOW, (0, 460), (WIDTH, 460), 4)
    for x in range(0, WIDTH, 80):
        pygame.draw.line(screen, WHITE, (x, 405), (x+40, 405), 3)
    pygame.draw.rect(screen, (185, 190, 190), UNLOADING_ZONE)
    pygame.draw.rect(screen, YELLOW, UNLOADING_ZONE, 4)
    for x in range(UNLOADING_ZONE.left + 12, UNLOADING_ZONE.right - 15, 25):
        pygame.draw.line(screen, YELLOW, (x, UNLOADING_ZONE.top + 8), (x + 18, UNLOADING_ZONE.bottom - 8), 2)
    screen.blit(font.render("UNLOADING ZONE", True, BLACK), (UNLOADING_ZONE.left + 35, UNLOADING_ZONE.top + 8))
    pygame.draw.rect(screen, (185, 190, 190), LOADING_ZONE)
    pygame.draw.rect(screen, YELLOW, LOADING_ZONE, 4)
    pygame.draw.rect(screen, (55, 95, 115), CHARGING_STATION, border_radius=8)
    pygame.draw.rect(screen, WHITE, CHARGING_STATION, 3, border_radius=8)
    screen.blit(small.render("CHARGING STATION", True, WHITE),
                (CHARGING_STATION.x + 18, CHARGING_STATION.y + 7))
    for bay_x, label in ((AMR1_CHARGE_POINT[0], "01"), (AMR2_CHARGE_POINT[0], "02")):
        bay = pygame.Rect(int(bay_x - 27), int(AMR1_CHARGE_POINT[1] - 20), 54, 48)
        pygame.draw.rect(screen, (115, 145, 155), bay, 2, border_radius=5)
        screen.blit(tiny.render(f"BAY {label}", True, WHITE), (bay.x + 8, bay.y + 15))
    for x in range(1110, 1240, 25):
        pygame.draw.line(screen, YELLOW, (x, 615), (x+30, 755), 2)
    screen.blit(font.render("LOADING ZONE", True, BLACK), (1105, 735))
    pygame.draw.rect(screen, DARK, (2, 2, WIDTH-4, HEIGHT-4), 5)


def draw_rack(rect):
    pygame.draw.rect(screen, DARK, rect.move(5, 5), border_radius=4)
    pygame.draw.rect(screen, STEEL, rect, border_radius=4)
    for y in range(rect.top+10, rect.bottom-5, 48):
        pygame.draw.rect(screen, DARK_BLUE, (rect.left+5, y, rect.width-10, 5))
        for x in range(rect.left+12, rect.right-25, 42):
            pallet = pygame.Rect(x, y+7, 30, 30)
            pygame.draw.rect(screen, WOOD, pallet)
            pygame.draw.rect(screen, BLACK, pallet, 1)
            pygame.draw.line(screen, BROWN, (x+10, y+7), (x+10, y+37), 2)
            pygame.draw.line(screen, BROWN, (x+20, y+7), (x+20, y+37), 2)
    for x in (rect.left+3, rect.right-7):
        pygame.draw.rect(screen, YELLOW, (x, rect.top, 5, rect.height))
    pygame.draw.rect(screen, BLACK, rect, 2)


def draw_equipment():
    for item in obstacles:
        r, kind = item["rect"], item["kind"]
        if kind == "rack":
            draw_rack(r)
        elif kind == "machine":
            pygame.draw.rect(screen, DARK, r.move(5,5), border_radius=8)
            pygame.draw.rect(screen, BLUE, r, border_radius=8)
            pygame.draw.rect(screen, BLACK, r, 3, border_radius=8)
            panel = pygame.Rect(r.x+12, r.y+10, r.width-24, 25)
            pygame.draw.rect(screen, DARK_BLUE, panel, border_radius=4)
            for i, c in enumerate((GREEN, YELLOW, RED)):
                pygame.draw.circle(screen, c, (panel.left+18+i*25, panel.centery), 5)
            pygame.draw.rect(screen, LIGHT, (r.x+15, r.y+48, r.width-30, r.height-60), border_radius=5)
        elif kind == "pallet":
            pygame.draw.rect(screen, WOOD, r, border_radius=3)
            pygame.draw.rect(screen, BLACK, r, 2, border_radius=3)
            for y in range(r.top+12, r.bottom, 18):
                pygame.draw.line(screen, BROWN, (r.left+3,y), (r.right-3,y), 3)
    for r in dynamic_obstacles:
        pygame.draw.circle(screen, ORANGE, r.center, 17)
        pygame.draw.circle(screen, BLACK, r.center, 17, 2)
    for item in moving_obstacles:
        r = item["rect"]
        if item["kind"] == "forklift":
            pygame.draw.rect(screen, ORANGE, r, border_radius=5)
            pygame.draw.rect(screen, BLACK, r, 2, border_radius=5)
            pygame.draw.rect(screen, DARK_BLUE, (r.x+12,r.y+4,18,15))
            pygame.draw.circle(screen, BLACK, (r.left+8,r.bottom-2), 5)
            pygame.draw.circle(screen, BLACK, (r.right-8,r.bottom-2), 5)
        else:
            pygame.draw.circle(screen, (230,190,150), r.center, 8)
            pygame.draw.circle(screen, RED, (r.centerx,r.centery+9), 7)


def draw_crate(pos, size=24):
    x, y = int(pos[0]), int(pos[1])
    r = pygame.Rect(x-size//2, y-size//2, size, size)
    pygame.draw.rect(screen, WOOD, r, border_radius=3)
    pygame.draw.rect(screen, BLACK, r, 2, border_radius=3)
    pygame.draw.line(screen, YELLOW, (r.centerx,r.top+2), (r.centerx,r.bottom-2), 3)
    pygame.draw.line(screen, YELLOW, (r.left+2,r.centery), (r.right-2,r.centery), 2)


def draw_packages():
    for i, item in enumerate(amr2_stock):
        if item["state"] == "in_unloading":
            draw_crate(UNLOADING_SLOTS[i])
        elif item["state"] == "delivered":
            draw_crate(RACK_DROP_POINTS[i], 26)
    for i, p in enumerate(packages):
        if p["state"] == "at_pickup":
            draw_crate(PICKUPS[i])
            screen.blit(small.render(f"P{i+1}", True, BLACK), (PICKUPS[i][0]-12, PICKUPS[i][1]-32))
        elif p["state"] == "in_loading":
            draw_crate(LOADING_SLOTS[i])
        elif p["state"] == "returned":
            pygame.draw.circle(screen, GREEN, (int(PICKUPS[i][0]),int(PICKUPS[i][1])), 10)
            pygame.draw.line(screen, WHITE, (PICKUPS[i][0]-5,PICKUPS[i][1]), (PICKUPS[i][0]-1,PICKUPS[i][1]+5), 2)
            pygame.draw.line(screen, WHITE, (PICKUPS[i][0]-1,PICKUPS[i][1]+5), (PICKUPS[i][0]+7,PICKUPS[i][1]-6), 2)
    # Packages currently carried are drawn on top of their AMR bodies.


def draw_robot(robot, number, color):
    surf = pygame.Surface((56,54), pygame.SRCALPHA)
    pygame.draw.rect(surf, color, (5,9,46,36), border_radius=9)
    pygame.draw.rect(surf, BLACK, (5,9,46,36), 2, border_radius=9)
    pygame.draw.rect(surf, BLACK, (0,14,8,26), border_radius=3)
    pygame.draw.rect(surf, BLACK, (48,14,8,26), border_radius=3)
    pygame.draw.polygon(surf, YELLOW, [(43,18),(54,27),(43,36)])
    pygame.draw.circle(surf, WHITE, (28,27), 5)
    pygame.draw.circle(surf, RED, (28,27), 2)
    if robot["carrying"] is not None:
        pygame.draw.rect(surf, WOOD, (17,9,22,20), border_radius=3)
        pygame.draw.rect(surf, BLACK, (17,9,22,20), 2, border_radius=3)
    rotated = pygame.transform.rotate(surf, robot["angle"])
    rect = rotated.get_rect(center=(int(robot["x"]),int(robot["y"])))
    screen.blit(rotated, rect)
    screen.blit(small.render(f"AMR-{number:02d}", True, BLACK), (int(robot["x"])-27,int(robot["y"])+32))
    battery = max(0.0, min(100.0, robot.get("battery", 100.0)))
    bar_x, bar_y, bar_w, bar_h = int(robot["x"])-22, int(robot["y"])-34, 44, 7
    pygame.draw.rect(screen, BLACK, (bar_x, bar_y, bar_w, bar_h), border_radius=2)
    fill_color = GREEN if battery > 50 else ORANGE if battery > 20 else RED
    pygame.draw.rect(screen, fill_color,
                     (bar_x+1, bar_y+1, int((bar_w-2)*battery/100), bar_h-2), border_radius=2)
    screen.blit(tiny.render(f"{battery:.0f}%", True, BLACK), (bar_x+12, bar_y-15))


def draw_route(robot, color):
    if robot["path"]:
        pts = [(int(robot["x"]),int(robot["y"]))] + [(int(x),int(y)) for x,y in robot["path"]]
        if len(pts) > 1:
            pygame.draw.lines(screen, color, False, pts, 3)
    if robot["goal"]:
        pygame.draw.circle(screen, color, (int(robot["goal"][0]),int(robot["goal"][1])), 7, 2)


def draw_trail(robot, color):
    if len(robot["trail"]) > 1:
        pygame.draw.lines(screen, color, False, [(int(x),int(y)) for x,y in robot["trail"]], 2)


def draw_sensor_lines(robot):
    # Sensors are visual only; route safety is enforced by collision checks + replanning.
    for rel in sensor_angles:
        end, _, hit = sensor_hit(robot, robot["angle"] + rel)
        start = (robot["x"] + math.cos(math.radians(robot["angle"]+rel))*ROBOT_RADIUS,
                 robot["y"] - math.sin(math.radians(robot["angle"]+rel))*ROBOT_RADIUS)
        pygame.draw.line(screen, RED if hit else GREEN, start, end, 1)


def draw_button(rect, label):
    pygame.draw.rect(screen, DARK_BLUE, rect, border_radius=8)
    pygame.draw.rect(screen, WHITE, rect, 2, border_radius=8)
    for y in (rect.y+9, rect.y+17, rect.y+25):
        pygame.draw.circle(screen, WHITE, (rect.centerx,y), 2)


def fleet_alerts():
    """Build the current live alert list from fleet telemetry and task states."""
    alerts = []
    separation = distance(robot_pos(amr1), robot_pos(amr2))
    if separation < SAFE_DISTANCE:
        alerts.append((RED, "EMERGENCY: AMR separation below safety threshold"))

    for label, robot, task in (("AMR-01", amr1, amr1_task), ("AMR-02", amr2, amr2_task)):
        battery = robot.get("battery", 100.0)
        if battery <= 20:
            alerts.append((RED if battery <= 10 else ORANGE,
                           f"{label}: Critical battery level ({battery:.0f}%)"))
        if task in ("WAITING_FOR_ROUTE", "WAITING_FOR_CHARGER"):
            alerts.append((ORANGE, f"{label}: Mission delayed - route/charger unavailable"))
        status = robot.get("status", "")
        if any(word in status.lower() for word in ("blocked", "failure", "cannot", "collision")):
            alerts.append((ORANGE, f"{label}: {status}"))
        elif "replanning" in status.lower():
            alerts.append((YELLOW, f"{label}: {status}"))

    if amr1_task == "CANCELLED":
        alerts.append((ORANGE, "AMR-01 mission was cancelled by operator"))
    return alerts


def draw_fleet_dashboard():
    """Render the centralized live fleet dashboard."""
    pygame.draw.rect(screen, (20, 31, 43), fleet_panel, border_radius=12)
    pygame.draw.rect(screen, (105, 160, 195), fleet_panel, 2, border_radius=12)
    screen.blit(title_font.render("CENTRALIZED FLEET MANAGEMENT", True, WHITE),
                (fleet_panel.x + 16, fleet_panel.y + 12))
    screen.blit(tiny.render("LIVE TELEMETRY  |  TWO-ROBOT WAREHOUSE FLEET", True, LIGHT),
                (fleet_panel.x + 18, fleet_panel.y + 40))

    # Mission allocation and aggregate completion.
    mission_active = amr1_task not in ("IDLE", "CANCELLED", "COMPLETE") or amr2_started
    mission_label = "MISSION ACTIVE" if mission_active else "MISSION NOT STARTED"
    mission_color = GREEN if mission_active else YELLOW
    screen.blit(small.render(f"FLEET DISPATCH: {mission_label}", True, mission_color),
                (fleet_panel.x + 18, fleet_panel.y + 65))
    amr1_done = sum(p["state"] == "in_loading" for p in packages)
    amr2_done = sum(p["state"] == "delivered" for p in amr2_stock)
    screen.blit(small.render(f"AMR-01 loading deliveries: {amr1_done}/6", True, WHITE),
                (fleet_panel.x + 18, fleet_panel.y + 91))
    screen.blit(small.render(f"AMR-02 rack deliveries: {amr2_done}/6", True, WHITE),
                (fleet_panel.x + 260, fleet_panel.y + 91))

    def robot_card(y, label, robot, task, color, completed, task_total):
        card = pygame.Rect(fleet_panel.x + 14, y, fleet_panel.width - 28, 126)
        pygame.draw.rect(screen, (37, 52, 67), card, border_radius=8)
        pygame.draw.rect(screen, color, card, 2, border_radius=8)
        screen.blit(small.render(label, True, color), (card.x + 12, card.y + 8))
        screen.blit(tiny.render(f"TASK: {task}", True, WHITE), (card.x + 112, card.y + 10))
        status = robot.get("status", "")
        if len(status) > 56:
            status = status[:53] + "..."
        screen.blit(tiny.render(f"STATUS: {status}", True, LIGHT), (card.x + 12, card.y + 34))
        screen.blit(tiny.render(f"POSITION: X {robot['x']:.1f}  Y {robot['y']:.1f}", True, WHITE),
                    (card.x + 12, card.y + 58))
        screen.blit(tiny.render(f"COMPLETION: {completed}/{task_total}", True, WHITE),
                    (card.x + 12, card.y + 82))
        battery = max(0.0, min(100.0, robot.get("battery", 100.0)))
        battery_color = GREEN if battery > 50 else ORANGE if battery > 20 else RED
        bar = pygame.Rect(card.x + 275, card.y + 84, 190, 13)
        pygame.draw.rect(screen, BLACK, bar, border_radius=5)
        fill = bar.copy()
        fill.width = int((bar.width - 2) * battery / 100)
        fill.x += 1
        fill.y += 1
        fill.height -= 2
        pygame.draw.rect(screen, battery_color, fill, border_radius=4)
        screen.blit(tiny.render(f"BATTERY {battery:.0f}%", True, WHITE),
                    (card.x + 275, card.y + 63))

    robot_card(fleet_panel.y + 119, "AMR-01", amr1, amr1_task, BLUE, amr1_done, 6)
    robot_card(fleet_panel.y + 253, "AMR-02", amr2, amr2_task, GREEN, amr2_done, 6)

    # Emergency and failure alerts are evaluated continuously from live state.
    alert_y = fleet_panel.y + 389
    screen.blit(small.render("EMERGENCY & FAILURE ALERTS", True, WHITE),
                (fleet_panel.x + 18, alert_y))
    alerts = fleet_alerts()
    if not alerts:
        screen.blit(tiny.render("NO ACTIVE ALERTS - fleet operating normally", True, GREEN),
                    (fleet_panel.x + 20, alert_y + 25))
    else:
        for i, (alert_color, message) in enumerate(alerts[:4]):
            if len(message) > 67:
                message = message[:64] + "..."
            screen.blit(tiny.render("! " + message, True, alert_color),
                        (fleet_panel.x + 20, alert_y + 23 + i * 20))

    # Dispatch controls are clickable, in addition to the keyboard shortcuts.
    for rect, label, fill_color in (
        (fleet_start_button, "DISPATCH / START FLEET (T)", (20, 105, 65)),
        (fleet_reset_button, "RESET SIMULATION (R)", (80, 65, 55)),
    ):
        pygame.draw.rect(screen, fill_color, rect, border_radius=7)
        pygame.draw.rect(screen, WHITE, rect, 1, border_radius=7)
        text_surface = tiny.render(label, True, WHITE)
        screen.blit(text_surface, text_surface.get_rect(center=rect.center))


def draw_panels():
    draw_button(info_button, "")
    draw_button(controls_button, "")
    pygame.draw.rect(screen, (15, 85, 65), fleet_button, border_radius=8)
    pygame.draw.rect(screen, WHITE, fleet_button, 2, border_radius=8)
    fleet_label = tiny.render("FLEET [F]", True, WHITE)
    screen.blit(fleet_label, fleet_label.get_rect(center=fleet_button.center))
    if show_info:
        pygame.draw.rect(screen, DARK_BLUE, info_panel, border_radius=10)
        pygame.draw.rect(screen, WHITE, info_panel, 2, border_radius=10)
        screen.blit(title_font.render("WAREHOUSE INFORMATION", True, WHITE), (info_panel.x+14,info_panel.y+10))
        rows = [
            f"AMR-01: {amr1_task} | {amr1['status']}",
            f"AMR-01 position: ({amr1['x']:.0f}, {amr1['y']:.0f})",
            f"AMR-01 distance: {amr1['distance']:.1f} px | Battery: {amr1['battery']:.0f}%",
            f"AMR-01 delivered to loading zone: {amr1_completed}/6",
            f"AMR-02: {amr2_task} | {amr2['status']}",
            f"AMR-02 position: ({amr2['x']:.0f}, {amr2['y']:.0f})",
            f"AMR-02 distance: {amr2['distance']:.1f} px | Battery: {amr2['battery']:.0f}%",
            f"AMR-02 rack deliveries: {sum(p['state']=='delivered' for p in amr2_stock)}/6",
            f"Boxes in unloading zone: {sum(p['state']=='in_unloading' for p in amr2_stock)}",
            f"AMR-01 packages at loading zone: {sum(p['state']=='in_loading' for p in packages)}",
            f"Dynamic obstacles: {len(dynamic_obstacles)} | Forklift patrol active",
            f"Robot separation: {distance(robot_pos(amr1),robot_pos(amr2)):.1f} px",
            f"Safety state: {'COLLISION RISK' if distance(robot_pos(amr1),robot_pos(amr2)) < SAFE_DISTANCE else 'SAFE'}",
        ]
        for i, line in enumerate(rows):
            screen.blit(small.render(line, True, WHITE), (info_panel.x+14, info_panel.y+43+i*23))
    if show_controls:
        pygame.draw.rect(screen, DARK, controls_panel, border_radius=10)
        pygame.draw.rect(screen, WHITE, controls_panel, 2, border_radius=10)
        screen.blit(title_font.render("WAREHOUSE CONTROLS", True, WHITE), (controls_panel.x+12,controls_panel.y+10))
        lines = ["T : Start six-package fleet mission", "F : Open/close fleet dashboard", "X : Cancel AMR-01 mission", "R : Reset all packages / robots", "Left click : Manual AMR-01 goal", "Right click : Add obstacle", "W/S : Manual forward/backward", "A/D : Rotate AMR-01", "SPACE : Cancel manual route", "AMR-02 : Unloading zone to six racks", "AMR-02 starts with AMR-01", "Auto-charge: AMR-01 after box 4", "Auto-charge: AMR-02 after box 3", "ESC : Exit"]
        for i, line in enumerate(lines):
            screen.blit(small.render(line, True, WHITE), (controls_panel.x+12,controls_panel.y+44+i*25))
    if show_fleet:
        draw_fleet_dashboard()


def draw_status():
    p1 = f"AMR-01: {amr1_task} | Delivered to loading: {amr1_completed}/6"
    p2 = f"AMR-02: {amr2_task} | Rack deliveries: {sum(p['state']=='delivered' for p in amr2_stock)}/6"
    screen.blit(small.render(p1, True, BLACK), (18, HEIGHT-62))
    screen.blit(small.render(p2, True, BLACK), (18, HEIGHT-40))
    sep = distance(robot_pos(amr1), robot_pos(amr2))
    safety = RED if sep < SAFE_DISTANCE else GREEN
    screen.blit(small.render(f"Robot separation: {sep:.1f}px | {'COLLISION RISK' if sep < SAFE_DISTANCE else 'SAFE'}", True, safety), (18, HEIGHT-18))

# ------------------------- obstacle placement / reset -------------------------
def add_dynamic_obstacle(x, y):
    new = pygame.Rect((x//CELL)*CELL+3, (y//CELL)*CELL+3, CELL-6, CELL-6)
    if new.colliderect(LOADING_ZONE) or new.colliderect(UNLOADING_ZONE) or any(new.colliderect(o["rect"]) for o in obstacles) or any(new.colliderect(o["rect"]) for o in moving_obstacles):
        amr1["status"] = "Cannot place obstacle there"
        return
    for r in (amr1, amr2):
        if new.inflate(ROBOT_RADIUS*2,ROBOT_RADIUS*2).collidepoint(r["x"],r["y"]):
            amr1["status"] = "Cannot place obstacle on a robot"
            return
    dynamic_obstacles.append(new)
    # Both autonomous routes are recalculated after a new obstacle appears.
    if amr1["goal"] is not None:
        replan_robot(amr1, amr2, False)
    if amr2["goal"] is not None and amr2_started:
        replan_robot(amr2, amr1, True)


def reset_simulation():
    global amr1_task, amr1_index, amr1_timer, amr1_payload_progress, amr1_completed, amr1_charge_timer
    global amr2_started, amr2_queue, amr2_task, amr2_index, amr2_timer, amr2_payload_progress, amr2_charge_timer
    global manual_goal, manual_path, dynamic_obstacles
    for robot, x, y in ((amr1,80.0,80.0),(amr2,80.0,720.0)):
        robot.update({"x":x,"y":y,"angle":0.0,"path":[],"goal":None,"status":"Waiting for task","distance":0.0,"trail":[(x,y)],"last_trail":(x,y),"carrying":None,"battery":100.0})
    for p in packages:
        p["state"] = "at_pickup"
        p["slot"] = None
    for item in amr2_stock:
        item["state"] = "in_unloading"
    amr1_task, amr1_index, amr1_timer, amr1_payload_progress, amr1_completed = "IDLE", None, 0.0, 0.0, 0
    amr1_charge_timer = 0.0
    amr2_started, amr2_queue, amr2_task, amr2_index = False, deque(), "STANDBY", None
    amr2_timer, amr2_payload_progress = 0.0, 0.0
    amr2_charge_timer = 0.0
    manual_goal, manual_path = None, []
    dynamic_obstacles = []

# ------------------------- main loop -------------------------
running = True
while running:
    dt = min(clock.tick(FPS) / 1000.0, 0.04)
    sensor_cooldown = max(0.0, sensor_cooldown-dt)
    update_moving_obstacles(dt)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_t:
                start_amr1_sequence()
            elif event.key == pygame.K_x:
                amr1_task = "CANCELLED"
                amr1["path"] = []
                amr1["goal"] = None
                amr1["carrying"] = None
                amr1["status"] = "Mission cancelled"
            elif event.key == pygame.K_r:
                reset_simulation()
            elif event.key == pygame.K_f:
                show_fleet = not show_fleet
            elif event.key == pygame.K_SPACE:
                if amr1_task in ("IDLE", "CANCELLED", "COMPLETE"):
                    manual_path = []
                    manual_goal = None
                    amr1["path"] = []
                    amr1["goal"] = None
                    amr1["status"] = "Manual navigation cancelled"
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos
            if event.button == 1:
                if info_button.collidepoint(mx,my):
                    show_info = not show_info
                elif controls_button.collidepoint(mx,my):
                    show_controls = not show_controls
                elif fleet_button.collidepoint(mx,my):
                    show_fleet = not show_fleet
                elif show_fleet and fleet_start_button.collidepoint(mx,my):
                    start_amr1_sequence()
                elif show_fleet and fleet_reset_button.collidepoint(mx,my):
                    reset_simulation()
                elif amr1_task in ("IDLE", "CANCELLED", "COMPLETE"):
                    manual_goal = (float(mx),float(my))
                    if point_is_clear(mx,my):
                        set_robot_goal(amr1, manual_goal, amr2 if amr2_started else None, False)
                        robot1_manual = True
                    else:
                        amr1["status"] = "Invalid destination - blocked"
            elif event.button == 3:
                add_dynamic_obstacle(mx,my)

    keys = pygame.key.get_pressed()
    # Autonomous AMR-01 mission. Manual drive is available only outside a mission.
    if amr1_task not in ("IDLE", "CANCELLED", "COMPLETE"):
        if amr1["path"]:
            move_robot(amr1, amr2, dt, False)
        update_amr1_task(dt)
        # Retry a temporarily blocked route instead of leaving AMR-01 stranded.
        if not amr1["path"] and amr1["goal"] is not None and distance(robot_pos(amr1), amr1["goal"]) > 14:
            replan_robot(amr1, amr2 if amr2_started else None, False)
    else:
        if amr1["path"]:
            move_robot(amr1, amr2, dt, False)
        else:
            turn = (keys[pygame.K_a] - keys[pygame.K_d]) * ROTATION_SPEED * dt
            amr1["angle"] += turn
            drive = (keys[pygame.K_w] - keys[pygame.K_s]) * AMR1_SPEED * dt
            if drive:
                nx = amr1["x"] + drive*math.cos(math.radians(amr1["angle"]))
                ny = amr1["y"] - drive*math.sin(math.radians(amr1["angle"]))
                if not collision_at(nx,ny) and movement_safe(amr1,nx,ny,amr2):
                    amr1["x"],amr1["y"] = nx,ny

    # AMR-02 is entirely autonomous and starts with AMR-01 when T is pressed.
    if amr2_started:
        if amr2["path"]:
            move_robot(amr2, amr1, dt, True)
        update_amr2_task(dt)
        # Replan if a moving hazard temporarily emptied AMR-02's route.
        if (not amr2["path"] and amr2["goal"] is not None
                and distance(robot_pos(amr2), amr2["goal"]) > 14
                and amr2_task not in ("WAITING_FOR_PACKAGE", "WAITING_FOR_ROUTE", "WAITING_FOR_CHARGER", "CHARGING")):
            if not set_robot_goal(amr2, amr2["goal"], amr1, True):
                amr2_task = "WAITING_FOR_CHARGER" if amr2_task == "TO_CHARGER" else "WAITING_FOR_ROUTE"
        if amr2_task in ("WAITING_FOR_ROUTE", "WAITING_FOR_CHARGER") and amr2_index is not None:
            update_amr2_task(dt)

    update_trail(amr1)
    update_trail(amr2)

    # ------------------------- render -------------------------
    draw_floor()
    draw_equipment()
    draw_packages()
    draw_trail(amr1, TRAIL_BLUE)
    draw_trail(amr2, (20,150,90))
    draw_route(amr1, YELLOW)
    draw_route(amr2, GREEN)
    draw_robot(amr1, 1, BLUE)
    draw_robot(amr2, 2, GREEN)
    # Safety state and emergency alerts are presented in the fleet dashboard.
    if amr1["carrying"] is not None:
        screen.blit(tiny.render(f"P{amr1['carrying']+1}", True, BLACK), (int(amr1["x"])+18,int(amr1["y"])-28))
    if amr2["carrying"] is not None:
        screen.blit(tiny.render(f"P{amr2['carrying']+1}", True, BLACK), (int(amr2["x"])+18,int(amr2["y"])-28))
    draw_panels()
    draw_status()
    pygame.display.flip()

pygame.quit()
