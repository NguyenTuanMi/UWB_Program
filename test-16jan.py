#Program for path finding and alignment. 
#Written by Vincent Gaudeo Lie
#from improved_obstacle_capture_v1 import *
from matplotlib import pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import Polygon
import heapq
from math import sqrt, pi, cos, sin, ceil
from time import sleep, time
from shapely import Polygon as Poly_shapely
import random
import math
from djitellopy import Tello
from UWB_Manipulation.UWB_Reader import get_target_position
from shared_utils.customtello import CustomTello
from swarmserver.swarmserverclientnew_demo import MarkerClient

pi_id = 6
network_config = {
            'host': f'192.168.0.{100+pi_id}',
            'control_port': 9000 + pi_id,
            #'control_port': 9020 + pi_id,
            'state_port': 8000 + pi_id,
            'video_port': 11100 + pi_id
}

USE_LAST_MAP = True
if USE_LAST_MAP:
    with open("last_save.txt", "r") as file:
        exec(file.read())

with open("last_save.txt", "w") as file:
    file.write(f"notsafe_obs_edges = {notsafe_obs_edges}\n")
    file.write(f"safe_area = {safe_area}\n")
    file.write(f"wall_diag = {wall_diag}\n")

tello = CustomTello(network_config=network_config)
tello.connect()
print(tello.get_battery())
initial_yaw = tello.get_yaw()
def enlarge(vertices, buffer = 30):
    poll = Poly_shapely(vertices)
    buffered_polygon = poll.buffer(buffer)
    vert = tuple(buffered_polygon.exterior.coords)
    return vert

initial_start = time()
class node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    banned = False
    sudobanned = False
    in_safe_area = False
    edge_safe_area = False
    neighbours = list()

fig, ax = plt.subplots(1,1, dpi=130)
DIST_TOLERANCE = 50
MAX_MOVE = 100
START_POINT = (48, 608)
DELAY_GO = 0
increment = 4
turn_penalty = 20
first_search_radius = sqrt(2) * 2 * increment
diag_length = sqrt(2) * increment
visited_threshold = increment * 2
dx = [0, 0, 1, -1, -1, -1, 1, 1]
dy = [1, -1, 0, 0, 1, -1, 1, -1]
nodes = []
safe_area_path = Path(safe_area)
safe_area = enlarge(safe_area, -7)
safe_area_unlarge = Path(safe_area)
random_colors = ["white", "black", "red", "green", "purple", "blue"]
obs_edges = [enlarge(obs) for obs in notsafe_obs_edges]
unsafe_paths = [Path(obs) for obs in notsafe_obs_edges]
obs_paths = [Path(obs) for obs in obs_edges]
if wall_diag[0][0] > wall_diag[1][0]:
    wall_diag = [wall_diag[1], wall_diag[0]]
wall_coord = [wall_diag[0], (wall_diag[0][0],wall_diag[1][1]),
              wall_diag[1], (wall_diag[1][0],wall_diag[0][1])]
wall_size = [wall_diag[1][0]-wall_diag[0][0], wall_diag[1][1]-wall_diag[0][1]]
offset_factor = 0.03
offset = [wall_size[0]*offset_factor, wall_size[1]*offset_factor]
xlim = [wall_diag[0][0]-offset[0], wall_diag[1][0]+offset[0]]
ylim = wall_diag[0][1]-offset[1], wall_diag[1][1]+offset[1]
wall_path = Path(wall_coord)
wall_poly = Polygon(wall_coord, facecolor = '#f2c7c7')

ax.add_patch(wall_poly)
plt.xlim(*xlim)
plt.ylim(*ylim)

ax.set_aspect('equal')
plt.tight_layout()
x = wall_diag[0][0]
while x < wall_diag[1][0]:
    nodes.append([])
    y = wall_diag[0][1]
    while y < wall_diag[1][1]:
        nodes[-1].append(node(x,y))
        larger = safe_area_path.contains_point((x,y))
        smaller = safe_area_unlarge.contains_point((x,y))
        if larger:
            nodes[-1][-1].in_safe_area = True
            if not smaller:
                nodes[-1][-1].edge_safe_area = True
        for pt in obs_paths:
            if pt.contains_point((x,y)):
                nodes[-1][-1].sudobanned = True
                break
        for pt in unsafe_paths:
            if pt.contains_point((x,y)):
                nodes[-1][-1].banned = True
                break
        y += increment
    x += increment

for i, subnodes in enumerate(nodes):
    for j, nd in enumerate(subnodes):
        col = 'g'
        if nd.banned: col = 'r'
        elif nd.sudobanned: col = 'purple'
        elif not nd.in_safe_area: continue
        if nd.edge_safe_area: 
            col = 'blue'
        ax.plot(nd.x, nd.y, marker = 'o', color=col, markersize = 1)
s = []
def find_path(start, end = None, mode = "ptp", safe_y = -1):
    heap = []
    for i in range(len(nodes)):
        for j in range(len(nodes[i])):
            dist = sqrt((nodes[i][j].x-start[0])**2 + (nodes[i][j].y-start[1])**2)
            if dist < first_search_radius:
                if mode == "ptp":
                    h = sqrt((nodes[i][j].x-end[0])**2 + (nodes[i][j].y-end[1])**2)
                else:
                    h = nodes[i][j].y - safe_y
                heapq.heappush(heap, (dist+h, dist, i, j, (-1, -1), -1, -1))
    visited = dict()
    parent = dict()
    last_now = []
    while True:
        try:
            now = heapq.heappop(heap)
        except:
            now = last_now
            path = []
            par = parent[(now[2], now[3], now[-1]%10)]
            while par != (-1, -1, 9, -1):
                path.append(par)
                par = parent[(par[0], par[1], par[-1])]
            return path
        #print(f"Proc {now}")
        #ax.plot(nodes[now[2]][now[3]].x, nodes[now[2]][now[3]].y, marker='o', color='yellow', markersize=4)
        last_now = now
        if visited.get((now[2], now[3], now[-1]%10)): continue
        visited[(now[2], now[3], now[-1]%10)] = True
        parent[(now[2], now[3], now[-1]%10)] = now[4] + (now[-1]%10, now[-2])
        nd_now = nodes[now[2]][now[3]]
        x, y = nd_now.x, nd_now.y
        if mode == "ptp":
            dist_to_end = sqrt((end[0]-x)**2 + (end[1]-y)**2)
        else:
            dist_to_end = y - safe_y
        s.append(dist_to_end)
        if dist_to_end < visited_threshold:
            path = []
            par = parent[(now[2], now[3], now[-1]%10)]
            while par != (-1, -1, 9, -1):
                path.append(par)
                par = parent[(par[0], par[1], par[-1])]
            return path
        for k in range(8):
            try:
                ci, cj = now[2]+dx[k], now[3]+dy[k]
                if min(ci, cj) < 0: continue
                c_node = nodes[ci][cj]
                if c_node.banned or c_node.sudobanned or visited.get((ci, cj, k)):
                    continue
                head = k
                penalty = 0
                if now[-1] != -1 and ((now[-1]%10) != k):
                    if (now[-1]//10) < 5:
                        penalty = turn_penalty
                    else:
                        if (now[-1]%10) == k: head = now[-1] + 10
                        else: head += 1
                dist = now[1] + (diag_length if k >= 4 else increment)
                if mode == "ptp":
                    h = sqrt((c_node.x-end[0])**2 + (c_node.y-end[1])**2)
                else:
                    h = c_node.y - safe_y
                heapq.heappush(heap, (dist+h+penalty, dist, ci, cj, (now[2], now[3]), now[-1]%10, head))
                #print(f"Add {(ci, cj)}, len heap {len(heap)}")
            except:
                pass
initial_end = time()
print(f"Initialization time: {initial_end - initial_start}")
def getpos(target_id = 7, max_retries = 150):
    pos = get_target_position(target_id, max_retries)
    yww = [tello.get_yaw() for _ in range(15)]
    yww = yww[3:-3]
    yaw = sum(yww)/len(yww)
    yaw = int(yaw)
    return int(pos[0]*100), int(pos[1]*100), (360 + yaw - initial_yaw + 90) % 360
def getStartPoint():
    return getpos()
START_POINT = getStartPoint()[:2]
print("Start:", START_POINT)
def move_to(x, y):
    print(f"Called with {x}, {y}")
    cur_x, cur_y, heading = getpos()
    dist = sqrt((cur_x - x)**2 + (cur_y - y)**2)
    while dist > DIST_TOLERANCE:
        heading = (heading + 2*(270-heading))%360
        x_diff = x - cur_x
        y_diff = y - cur_y
        print(f"{x_diff=}, {y_diff=}, {heading=}")
        a = x_diff
        b = y_diff
        print(f"prenerf {a=}, {b=}")
        nerf = max(1, ceil(max(abs(a),abs(b)) / MAX_MOVE))
        a = int(a / nerf)
        b = int(b / nerf)
        print(f"a: {a}, b: {b}, dist: {dist}")
        if a == 0:
            if b > 0: angle = 90
            else: angle = 270
        else:
            angle = int(math.atan(abs(b/a)) * 180 / pi)
            if a < 0:
                if b > 0: angle = 180 - angle
                else: angle = 180 + angle
            elif b < 0:
                angle = 360 - angle
        angle = (angle - heading + 360) % 360
        if 350 <= angle <= 360:
            angle = 0
        if angle <= 180: tello.rotate_counter_clockwise(angle)
        else: tello.rotate_clockwise(360 - angle)
        tello.move_forward(int(math.sqrt(a*a + b*b)))
        #tello.go_xyz_speed(a, b, 0, 100)
        sleep(DELAY_GO)
        cur_x, cur_y, heading = getpos()
        dist = sqrt((cur_x - x)**2 + (cur_y - y)**2)
def draw_path(**kwargs):
    if not kwargs.get("given_path", True):
        path = find_path(kwargs['start'], kwargs['end'])
    else: path = kwargs['path']
    if path is None:
        print("No path available")
        s.sort()
        print(s[:20])
        return
    if kwargs.get("get_intermitten", False): result = []
    path.reverse()
    cur_dist = 0
    dist_threshold = 90

    start_batt = tello.get_battery()
    status_str = f"Waiting for takeoff. Start Batt %: {start_batt}"
    marker_client = MarkerClient(drone_id=pi_id, relay_task=True)
    marker_client.client_takeoff_simul([99], status_message=status_str)     # [99] ensures that takeoff is only triggered by the user. Otherwise, put the list of drone_ids you want to takeoff with.
    tello.takeoff()

    marker_client.wait_for_relay(tello)
    sleep(2)
    #tello.takeoff()
    for i, p in enumerate(path):
        nd = nodes[p[0]][p[1]]
        col = kwargs.get("path_color", "yellow")
        m_size = 3
        if (p[-1] != p[-2]) or (i == len(path)-1):
            m_size = 5
            col = "black"
            nx = round(nd.x)
            ny = round(nd.y)
            #Please replace the below line with the appropriate tello mover
            if i != 1 and kwargs.get('WANT_TO_MOVE', False): move_to(nx, ny)
            #ax.text(nd.x, nd.y, f"({nx}, {ny})", size = 6)
        if i in (0, len(path)-1):
            col = "red"
            m_size = 7
        if i > 0 and kwargs.get('draw_intermitten', False):
            prev_nd = nodes[path[i-1][0]][path[i-1][1]]
            cd = sqrt((prev_nd.x-nd.x)**2 + (prev_nd.y-nd.y)**2)
            cur_dist += cd
            if cur_dist > dist_threshold:
                m_size = 8
                col = 'green'
                cur_dist -= dist_threshold
                if kwargs.get("get_intermitten", False):
                    result.append((round(nd.x), round(nd.y)))
        ax.plot(nd.x, nd.y, marker = 'o', color = col, markersize = m_size)
    if kwargs.get("get_intermitten", False):
        return iter(result)

    tello.land()

#path = find_path(START_POINT, (70, 150))
mx = max(P[1] for P in safe_area)
mn = min(P[1] for P in safe_area)
print(mx, mn, int(mx - (mx-mn)*0.3))
path = find_path(START_POINT, mode = '--', safe_y = int(mn + (mx-mn)*0.3))
P = draw_path(path = path, get_intermitten = True, draw_intermitten = True, WANT_TO_MOVE = True)
print(f"Pathfinding time: {time() - initial_end}")
plt.show()