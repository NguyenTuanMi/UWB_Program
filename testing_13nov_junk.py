#Program for path finding and alignment. 
#Written by Vincent Gaudeo Lie
from matplotlib import pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import Polygon
import heapq
from math import sqrt, pi, cos, sin, ceil
from time import time, sleep
from shapely import Polygon as Poly_shapely
import random
import socket
import threading
from UWB_ReadUDP import get_target_position, get_all_positions
def enlarge(vertices, buffer = 0.7):
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
# Each drone: (TELLO_IP, TELLO_PORT, LOCAL_PORT)
DRONE_NUMS = [14, 16]
DRONES = [(f"192.168.0.1{x}", 9000+x, 9000+x) for x in DRONE_NUMS]
WANT_CLICK_INTERACT = True
DIST_TOLERANCE = 2
MAX_MOVE = 10
START_POINT = (19, 550)
DELAY_GO = 0
increment = 0.4
turn_penalty = 20
first_search_radius = sqrt(2) * increment
diag_length = sqrt(2) * increment
visited_threshold = increment * 2
dx = [0, 0, 1, -1, -1, 1, -1, 1]
dy = [1, -1, 0, 0, -1, 1, 1, -1]
nodes = []
notsafe_obs_edges = [
    [(249, 297), (247, 288), (234, 401), (185, 395)],
    [(128, 411), (82, 390), (55, 432), (130, 464)],
    [(223, 511), (167, 528), (186, 573), (242, 566)]
    # [(58.5, 160.5), (68.5, 160.5), (68.5, 163.5), (58.5, 163.5)], 
    # [(76.4, 185.1), (92.2, 185.1), (92.2, 188.1), (76.4, 188.1)], 
    # [(93.31, 198.46), (93.51, 191.01), (105.89, 190.71), (105.99, 193.63), (95.92, 194.03), (95.92, 198.46)],
    # [(64.63, 181.2), (72.27, 181.3), (73.08, 170.03), (70.86, 169.83), (70.46, 179.49), (64.73, 179.09)],
    # [(95.62, 181.76), (110.01, 182.56), (109.81, 176.12), (95.82, 175.01), (89.78, 178.74)],
    # [(76.1, 177.83), (75.9, 170.38), (86.46, 170.38), (86.26, 178.23)]
]
safe_area = [(-70, 192), (-99, 7), (324, -14), (338, 198)]
safe_area_path = Path(safe_area)
safe_area = enlarge(safe_area, -0.7)
safe_area_unlarge = Path(safe_area)
random_colors = ["white", "black", "red", "green", "purple", "blue"]
obs_edges = [enlarge(obs) for obs in notsafe_obs_edges]
unsafe_paths = [Path(obs) for obs in notsafe_obs_edges]
obs_paths = [Path(obs) for obs in obs_edges]
wall_diag = [(22, 622), (139, 89)]
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
# Create sockets for each drone
sockets = []
for ip, tello_port, local_port in DRONES:
    continue
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", local_port))
    sockets.append((sock, ip, tello_port))

# Function to receive responses for one socket
def receive_responses(sock, drone_ip):
    while True:
        try:
            response, ip = sock.recvfrom(1024)
            print(f"[{drone_ip}] Received: {response.decode()}")
        except OSError as e:
            if e.errno == 9:  # Socket closed
                break
            else:
                print(f"[{drone_ip}] Error: {e}")
                break

# Start receiving threads for each drone
for sock, ip, port in sockets:
    continue
    t = threading.Thread(target=receive_responses, args=(sock, ip))
    t.daemon = True
    t.start()
initial_commands = [("command", 1), ("motoron", 5), ("motoroff", 5), ("takeoff", 10)]
for cmd, delay in initial_commands:
    continue
    for sock, ip, port in sockets:
        try:
            sock.sendto(cmd.encode(), (ip, port))
            print(f"[{ip}] Sent: {cmd}")
        except Exception as e:
            print(f"[{ip}] Failed to send {cmd}: {e}")
    sleep(delay)
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
    while True:
        now = heapq.heappop(heap)
        #print(f"Proc {now}")
        #ax.plot(nodes[now[2]][now[3]].x, nodes[now[2]][now[3]].y, marker='o', color='yellow', markersize=4)
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
#ca, cb = int(START_POINT[0]), int(START_POINT[1]) #TESTING
def getpos(target_id = 1, max_retries = 15):
    pos = get_target_position(target_id, max_retries)
    return pos[0], pos[1], 0
def getStartPoint():
    pass
def move_to(x, y):
    print(f"Called with {x}, {y}")
    #global ca, cb #TESTING
    cur_x, cur_y, heading = getpos()
    dist = sqrt((cur_x - x)**2 + (cur_y - y)**2)
    print(dist)
    while dist > DIST_TOLERANCE:
        rad = heading * pi / 180.0
        x_diff = x - cur_x
        y_diff = y - cur_y
        a = x_diff * cos(rad) + y_diff * sin(rad)
        b = -x_diff * sin(rad) + y_diff * cos(rad)
        nerf = max(1, ceil(max(abs(a),abs(b)) / MAX_MOVE))
        a = int(a / nerf)
        b = int(b / nerf)
        cmd = f"go {a} {b} 0 100"
        sock, ip, port = sockets[0]
        try:
            continue
            sock.sendto(cmd.encode(), (ip, port))
            print(f"[{ip}] Sent: {cmd}")
        except Exception as e:
            print(f"[{ip}] Failed to send {cmd}: {e}")
        sleep(DELAY_GO)
        ca += a*cos(rad)-b*sin(rad)
        cb += a*sin(rad)+b*cos(rad)
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
    dist_threshold = 10
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
            if i != 1 and kwargs["WANT_TO_MOVE"]: move_to(nx, ny)
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

#path = find_path(START_POINT, (70, 150))
path = find_path(START_POINT, mode = '1', safe_y = max(P[1] for P in safe_area) - 2)
P = draw_path(path = path, get_intermitten = True, draw_intermitten = True, WANT_TO_MOVE = True)
print(f"Pathfinding time: {time() - initial_end}")

L = []
def on_click(event):
    # Check if the click is within the plot area
    if event.inaxes is not None:
        x, y = round(event.xdata), round(event.ydata)
        L.append((x, y))
        print(f"Clicked at: ({x}, {y})")
        ca, cb = x, y
        ax.plot(event.xdata, event.ydata, 'ro')
        try: draw_path(path = find_path((x, y), next(P)), path_color = random.choice(random_colors), WANT_TO_MOVE = False)
        except: pass
        plt.draw()
    else:
        print("Click was outside the plot!")
if WANT_CLICK_INTERACT: fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
if WANT_CLICK_INTERACT and L != []: print(L)

# Close sockets
for sock, _, _ in sockets:
    sock.close()
