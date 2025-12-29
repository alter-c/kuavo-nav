import yaml
import numpy as np
from heapq import heappush, heappop
from collections import defaultdict

# ----------------------------
# 1. 定义类并加载 YAML 文件
# ----------------------------
class Point2DInfo:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class RouteInfo:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

class StationInfo:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

with open('route.yaml', 'r') as f:
    data = yaml.unsafe_load(f)

# 注意：YAML 中的顶层 key 是 'Stations' 和 'Routes'（首字母大写）
stations = data['Stations']
routes = data['Routes']

# ----------------------------
# 2. 构建 station 索引和位置数组
# ----------------------------
pos_to_id = {}
positions = {}  # id -> (x, y)

for s in stations:
    x, y = s.pose[0], s.pose[1]     # ← s.pose 而不是 s['pose']
    sid = s.id                      # ← s.id 而不是 s['id']
    positions[sid] = (x, y)
    key = (round(x, 6), round(y, 6))
    pos_to_id[key] = sid

all_ids = sorted(positions.keys())
id_to_index = {sid: i for i, sid in enumerate(all_ids)}
index_to_id = {i: sid for sid, i in id_to_index.items()}
node_positions = np.array([positions[sid] for sid in all_ids])  # shape: (N, 2)

# ----------------------------
# 3. 构建邻接表（基于 station ID）
# ----------------------------
adj = defaultdict(list)

for route in routes:
    # route.points 是 Point2DInfo 对象列表
    p1_raw = route.points[0].point   # ← .point 而不是 ['point']
    p2_raw = route.points[1].point   # ← 同上

    p1_key = (round(p1_raw[0], 6), round(p1_raw[1], 6))
    p2_key = (round(p2_raw[0], 6), round(p2_raw[1], 6))

    id1 = pos_to_id.get(p1_key)
    id2 = pos_to_id.get(p2_key)

    if id1 is None or id2 is None:
        continue

    idx1 = id_to_index[id1]
    idx2 = id_to_index[id2]

    cost = np.linalg.norm(node_positions[idx1] - node_positions[idx2])

    adj[idx1].append((idx2, cost))
    adj[idx2].append((idx1, cost))

# ----------------------------
# 4. A* 搜索
# ----------------------------
def heuristic(idx_a, idx_b):
    return np.linalg.norm(node_positions[idx_a] - node_positions[idx_b])

def a_star(start_id, goal_id):
    if start_id not in id_to_index or goal_id not in id_to_index:
        raise ValueError("Start or goal station ID not found")

    start_idx = id_to_index[start_id]
    goal_idx = id_to_index[goal_id]

    open_set = []
    heappush(open_set, (0.0, start_idx))

    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start_idx] = 0.0
    f_score = defaultdict(lambda: float('inf'))
    f_score[start_idx] = heuristic(start_idx, goal_idx)

    while open_set:
        _, current = heappop(open_set)

        if current == goal_idx:
            path_idx = []
            while current in came_from:
                path_idx.append(current)
                current = came_from[current]
            path_idx.append(start_idx)
            path_idx.reverse()
            return [index_to_id[i] for i in path_idx]

        for neighbor, cost in adj[current]:
            tentative_g = g_score[current] + cost
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_idx)
                heappush(open_set, (f_score[neighbor], neighbor))

    return []  # 无路径

# ----------------------------
# 5. 使用示例
# ----------------------------
path = a_star(start_id=1, goal_id=103)
print("Path (station IDs):", path)