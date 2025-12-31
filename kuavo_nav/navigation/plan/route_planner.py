import json
import math
import heapq

class RoutePlanner:
    def __init__(self, route_file, start_distance=1.0, end_distance=0.5):
        self._data= load_route(route_file)
        self._sd = start_distance
        self._ed = end_distance

        self._points = self._data['points']
        self._routes = self._data['routes']

        self.init_adj()

    def init_adj(self):
        """初始化邻接表"""
        self._adj = {}
        for route in self._routes.values():
            id1, id2 = route
            if id1 not in self._adj:
                self._adj[id1] = []
            if id2 not in self._adj:
                self._adj[id2] = []
            cost = math.hypot(
                self._points[id1][0] - self._points[id2][0],
                self._points[id1][1] - self._points[id2][1],
            )
            self._adj[id1].append((id2, cost))
            self._adj[id2].append((id1, cost))

    def plan(self, start, end) -> list:
        print(self._adj)
        sx, sy, _ = start
        ex, ey, _ = end

        def distance_to_point(item, refx, refy):
            _, (px, py) = item
            return math.hypot(px - refx, py - refy)
        
        # find nearest start point
        start_id, start_pose = min(
            self._points.items(),
            key=lambda item: distance_to_point(item, sx, sy)
        )
        print(f"Nearest start point: {start_id} at {start_pose}")
        if math.hypot(start_pose[0]-sx, start_pose[1]-sy) > self._sd:
            print("Start point too far from any route point.")
            return None
        
        # find nearest end point
        end_id, end_pose = min(
            self._points.items(),
            key=lambda item: distance_to_point(item, ex, ey)
        )
        print(f"Nearest end point: {end_id} at {end_pose}")
        if math.hypot(end_pose[0]-ex, end_pose[1]-ey) > self._ed:
            print("End point too far from any route point.")
            return None

        # A* search
        id_path = self.astar_search(start_id, end_id)
        print("A* path:", id_path)
        if id_path is None:
            return None

        coord_path = [tuple(self._points[id]) for id in id_path]
        coord_path.append((ex, ey))

        print("Final coord path:", coord_path)
        return coord_path

    def astar_search(self, start_id: str, end_id: str) -> list | None:
        # 启发函数：当前点到终点的距离
        def heuristic(id: str) -> float:
            x, y = self._points[id]
            ex, ey = self._points[end_id]
            return math.hypot(x - ex, y - ey)

        # open 集合：最小堆，元素是 (f_cost, id)
        open_heap = []
        heapq.heappush(open_heap, (0.0, start_id))

        # g_cost: 从起点到当前点的实际代价
        g_cost = {start_id: 0.0}

        # 记录前驱，用于回溯路径
        came_from: dict[str, str | None] = {start_id: None}

        while open_heap:
            _, current = heapq.heappop(open_heap)

            # 回溯路径
            if current == end_id:
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
        print("No path found between ", start_id, " and ", end_id)
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
