#Program for path finding and alighment. 
#Written by Vincent Gaudeo Lie
#Please read line 152
from matplotlib import pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import Polygon
import heapq
from math import sqrt
from time import time
from shapely import Polygon as Poly_shapely
def enlarge(vertices):
    poll = Poly_shapely(vertices)
    buffered_polygon = poll.buffer(0.7)
    vert = tuple(buffered_polygon.exterior.coords)
    return vert

initial_start = time()
class node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    banned = False
    sudobanned = False
    neighbours = list()
fig, ax = plt.subplots(1,1, dpi=130)
increment = 0.3
turn_penalty = 40
first_search_radius = sqrt(2) * increment
diag_length = sqrt(2) * increment
visited_threshold = increment * 2
dx = [0, 0, 1, -1, -1, 1, -1, 1]
dy = [1, -1, 0, 0, -1, 1, 1, -1]
nodes = []
notsafe_obs_edges = [
    [(58.5, 160.5), (68.5, 160.5), (68.5, 163.5), (58.5, 163.5)], 
    [(76.4, 185.1), (92.2, 185.1), (92.2, 188.1), (76.4, 188.1)], 
    [(93.31, 198.46), (93.51, 191.01), (105.89, 190.71), (105.99, 193.63), (95.92, 194.03), (95.92, 198.46)],
    [(64.63, 181.2), (72.27, 181.3), (73.08, 170.03), (70.86, 169.83), (70.46, 179.49), (64.73, 179.09)],
    [(95.62, 181.76), (110.01, 182.56), (109.81, 176.12), (95.82, 175.01), (89.78, 178.74)],
    [(76.1, 177.83), (75.9, 170.38), (86.46, 170.38), (86.26, 178.23)]
]
obs_edges = [enlarge(obs) for obs in notsafe_obs_edges]
unsafe_paths = [Path(obs) for obs in notsafe_obs_edges]
obs_paths = [Path(obs) for obs in obs_edges]
wall_diag = [(58.5,146.5), (110.4,198.6)]
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
        if col != 'g': ax.plot(nd.x, nd.y, marker = 'o', color=col, markersize = 1)
s = []
def find_path(start, end):
    heap = []
    for i in range(len(nodes)):
        for j in range(len(nodes[i])):
            dist = sqrt((nodes[i][j].x-start[0])**2 + (nodes[i][j].y-start[1])**2)
            if dist < first_search_radius:
                h = sqrt((nodes[i][j].x-end[0])**2 + (nodes[i][j].y-end[1])**2)
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
        dist_to_end = sqrt((end[0]-x)**2 + (end[1]-y)**2)
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
                h = sqrt((c_node.x-end[0])**2 + (c_node.y-end[1])**2)
                heapq.heappush(heap, (dist+h+penalty, dist, ci, cj, (now[2], now[3]), now[-1]%10, head))
                #print(f"Add {(ci, cj)}, len heap {len(heap)}")
            except:
                pass
initial_end = time()
print(f"Initialization time: {initial_end - initial_start}")

path = find_path((103, 196), (70, 150))
print(f"Pathfinding time: {time() - initial_end}")
try:
    for i, p in enumerate(reversed(path)):
        nd = nodes[p[0]][p[1]]
        col = 'yellow'
        m_size = 3
        if (p[-1] != p[-2]) or (i == len(path)-1):
            col = "black"
            nx = round(nd.x)
            ny = round(nd.y)
            #
            if i != 1: print(f"Goto {nx}, {ny}")
            #ax.text(nd.x, nd.y, f"({nx}, {ny})", size = 6)
        if i in (0, len(path)-1):
            col = "red"
            m_size = 5
        ax.plot(nd.x, nd.y, marker = 'o', color = col, markersize = m_size)
except:
    print("No path available")
    s.sort()
    print(s[:20])
plt.show()
