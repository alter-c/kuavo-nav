import json
import math
import heapq

class RoutePlanner:
    def __init__(self, route_file):
        self._data= load_route(route_file)

        self._points = self._data['points']
        self._routes = self._data['routes']

        self.init_adj()

    def init_adj(self):
        """初始化邻接表"""
        self._adj = {}
        for route in self._routes.values():
            pid1, pid2 = route
            if pid1 not in self._adj:
                self._adj[pid1] = []
            if pid2 not in self._adj:
                self._adj[pid2] = []
            cost = math.hypot(
                self._points[pid1][0] - self._points[pid2][0],
                self._points[pid1][1] - self._points[pid2][1],
            )
            self._adj[pid1].append((pid2, cost))
            self._adj[pid2].append((pid1, cost))

    def plan(self, start, end) -> list:
        print(self._adj)
        sx, sy, _ = start
        ex, ey, _ = end

        def distance_to_point(item, refx, refy):
            _, (px, py) = item
            return math.hypot(px - refx, py - refy)
        
        # find nearest start point
        start_pid, start_point = min(
            self._points.items(),
            key=lambda item: distance_to_point(item, sx, sy)
        )
        print(f"Nearest start point: {start_pid} at {start_point}")
        if math.hypot(start_point[0]-sx, start_point[1]-sy) > 1.0:
            print("Start point too far from any route point.")
            return None
        
        # find nearest end point
        end_pid, end_point = min(
            self._points.items(),
            key=lambda item: distance_to_point(item, ex, ey)
        )
        print(f"Nearest end point: {end_pid} at {end_point}")
        if math.hypot(end_point[0]-ex, end_point[1]-ey) > 1.0:
            print("End point too far from any route point.")
            return None

        # A* search
        pid_path = self.astar_search(start_pid, end_pid)
        print("A* path:", pid_path)
        if pid_path is None:
            return None

        # pid路径转换为坐标路径, 追加终点坐标
        coord_path = [tuple(self._points[pid]) for pid in pid_path]
        coord_path.append((ex, ey))

        print("Final coord path:", coord_path)
        return coord_path

    def astar_search(self, start_pid: str, end_pid: str) -> list | None:
        # 启发函数：当前点到终点的距离
        def heuristic(pid: str) -> float:
            x, y = self._points[pid]
            ex, ey = self._points[end_pid]
            return math.hypot(x - ex, y - ey)

        # open 集合：最小堆，元素是 (f_cost, pid)
        open_heap = []
        heapq.heappush(open_heap, (0.0, start_pid))

        # g_cost: 从起点到当前点的实际代价
        g_cost = {start_pid: 0.0}

        # 记录前驱，用于回溯路径
        came_from: dict[str, str | None] = {start_pid: None}

        while open_heap:
            _, current = heapq.heappop(open_heap)

            # 回溯路径
            if current == end_pid:
                return self._reconstruct_path(came_from, current)

            # 遍历临界点
            for neighbor, edge_cost in self._adj.get(current, []):
                tentative_g = g_cost[current] + edge_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_heap, (f, neighbor))

        # 不可达
        print("No path found between ", start_pid, " and ", end_pid)
        return None

    @staticmethod
    def _reconstruct_path(came_from: dict, current: str) -> list:
        path = [current]
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def load_route(file_path: str) -> dict:
    try:
        with open(file_path, 'r') as f:
            route_map = json.load(f)
    except Exception as e:
        print(f"Failed to load route map: {e}")
        route_map = None
    return route_map


if __name__ == "__main__":
    planner = RoutePlanner("/home/cdx/code/kuavo-nav/configs/route_map.json")
    route = planner.plan((3, 2, 0), (33.5, -18, 0))
    # print("Planned route:", route)
