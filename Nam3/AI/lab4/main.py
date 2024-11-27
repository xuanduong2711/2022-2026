from collections import defaultdict
from queue import PriorityQueue, Queue
import math
from matplotlib import pyplot as plt

class Point(object):
    def __init__(self, x, y, polygon_id=-1):
        self.x = x
        self.y = y
        self.polygon_id = polygon_id
        self.g = 0
        self.pre = None
    
    def rel(self, other, line):
        return line.d(self) * line.d(other) >= 0
    
    def can_see(self, other, line):
        l1 = self.line_to(line.p1)
        l2 = self.line_to(line.p2)
        d3 = line.d(self) * line.d(other) < 0
        d1 = other.rel(line.p2, l1)
        d2 = other.rel(line.p1, l2)
        return not (d1 and d2 and d3)
    
    def line_to(self, other):
        return Edge(self, other)
    
    def heuristic(self, other):
        return euclid_distance(self, other)
    
    def __eq__(self, point):
        return point and self.x == point.x and self.y == point.y
    
    def __ne__(self, point):
        return not self.__eq__(point)
    
    def __lt__(self, point):
        return hash(self) < hash(point)
    
    def __str__(self):
        return "(%d, %d)" % (self.x, self.y)
    
    def __hash__(self):
        return self.x.__hash__() ^ self.y.__hash__()
    
    def __repr__(self):
        return "(%d, %d)" % (self.x, self.y)
    
class Edge(object):
    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2
    
    def get_adjacent(self, point):
        if point == self.p1:
            return self.p2
        if point == self.p2:
            return self.p1
        
    def d(self, point):
        vect_a = Point(self.p2.x - self.p1.x, self.p2.y - self.p1.y)
        vect_n = Point(-vect_a.y, vect_a.x)
        return vect_n.x * (point.x - self.p1.x) + vect_n.y * (point.y - self.p1.y)

    def __str__(self):
        return "({}, {})".format(self.p1, self.p2)

    def __contains__(self, point):
        return self.p1 == point or self.p2 == point

    def __hash__(self):
        return self.p1.__hash__() ^ self.p2.__hash__()

    def __repr__(self):
        return "Edge({!r}, {!r})".format(self.p1, self.p2)    
    
class Graph:
    def __init__(self, polygons):
        self.graph = defaultdict(set)
        self.edges = set()
        self.polygons = defaultdict(set)
        pid = 0
        for polygon in polygons:
            if len(polygon) == 2:
                polygon.pop()
            if polygon[0] == polygon[-1]:
                self.add_point(polygon[0])
            else:
                for i, point in enumerate(polygon):
                    neighbor_point = polygon[(i + 1) % len(polygon)]
                    edge = Edge(point, neighbor_point)
                    if len(polygon) > 2:
                        point.polygon_id = pid
                        neighbor_point.polygon_id = pid
                        self.polygons[pid].add(edge)
                    self.add_edge(edge)
                if len(polygon) > 2:
                    pid += 1

    def get_adjacent_points(self, point):
        return list(filter(None.__ne__, [edge.get_adjacent(point) for edge in self.edges]))
    
    
    def can_see(self, start):
        see_list = list()
        cant_see_list = list()
        polygon_start= []
        for polygon in self.polygons:
            polygon_points = self.get_polygon_points(polygon)
            # Tìm min và max của tọa độ x và y
            min_point_x = min(point.x for point in polygon_points)
            min_point_y = min(point.y for point in polygon_points)
            max_point_x = max(point.x for point in polygon_points)
            max_point_y = max(point.y for point in polygon_points)

            for edge in self.polygons[polygon]:
                for point in self.get_points():


                    cant_see_list.append(start)
                    see_list.append(point) # Giả sủ start đều nhinf thấy moị điểm trên đa giác

                    if start in self.get_polygon_points(polygon):
                        polygon_start = self.get_polygon_points(polygon)
                        for poly_point in self.get_polygon_points(polygon):
                                if poly_point not in self.get_adjacent_points(start):
                                    cant_see_list.append(poly_point)   # Thêm những điểm không kề nó sẽ vào tập cant_see         

                    # Point vs start không thuộc cùng 1 đa giác thì
                    if point not in polygon_start: 
                        if not start.can_see(point, edge):
                            cant_see_list.append(point)
                
                # Khi pont năm trong 1 đa giác thì không điểm nào nhìn thấy nó (năm trong khi) 
                if point.x < max_point_x and point.x > min_point_x:
                    if point.y < max_point_y and point.y > min_point_y:
                        return []
                # Điều này luôn đúng vì các đa giác đã cho là đa giác lồi 
        see_set = set(see_list)
        cant_see_set = set(cant_see_list)
        # Loại bỏ đi những điểm không thể nhìn thì sẽ còn lại nhưng điểm nhìn thấy
        see_list = list(see_set - cant_see_set)
        return see_list


        
    def get_polygon_points(self, index):
        point_set = set()
        for edge in self.polygons[index]:
            point_set.add(edge.p1)
            point_set.add(edge.p2)
        return point_set
    
    def get_points(self):
        return list(self.graph)
    
    def get_edges(self):
        return list(self.edges)
    
    
    def add_point(self, point):
        self.graph[point].add(point)

    def add_edge(self, edge):
        self.graph[edge.p1].add(edge)
        self.graph[edge.p2].add(edge)
        self.edges.add(edge)

    def __contains__(self, item):
        if isinstance(item, Point):
            return item in self.graph
        if isinstance(item, Edge):
            return item in self.edges
        return False
    
    def __getitem__(self, point):
        if point in self.graph:
            return self.graph[point]
        return set()
        
    def __str__(self):
        res = ""
        for point in self.graph:
            res += "\n" + str(point) + ": "
            for edge in self.graph[point]:
                res += str(edge)
        return res
    
    def __repr__(self):
        return self.__str__()

    def h(self, point):
        heuristic = getattr(self, 'heuristic', None)
        if heuristic:
            return heuristic[point]
        else:
            return -1

def euclid_distance(point1, point2):
    return round(float(math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)), 3)
def BFS(graph, start, end):
    frontier = Queue()
    frontier.put(start)
    visited = {start}
    parent = {start: None}
    path = []
    while not frontier.empty():
        current = frontier.get()
        
        if current == end:
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]

        for next_node in graph.can_see(current):
            if next_node not in visited:
                frontier.put(next_node)
                visited.add(next_node)
                parent[next_node] = current
    return path

def DFS(graph, start, end):
    frontier = [(start, 0)]
    visited = {start}
    parent = {start: None}
    path = []
    while frontier:
        current, _ = frontier.pop()       
        if current == end:
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]

        visible_nodes = []
        for next_node in graph.can_see(current):
            if next_node not in visited:
                dist = euclid_distance(next_node, end)
                visible_nodes.append((next_node, dist))
        
        visible_nodes.sort(key=lambda x: x[1], reverse=True)
        
        for next_node, _ in visible_nodes:
            frontier.append((next_node, _))
            visited.add(next_node)
            parent[next_node] = current
    return path

def UCS(graph, start, end):
    frontier = PriorityQueue()
    frontier.put((0, start))
    visited = set()
    parent = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current_cost, current = frontier.get()
        
        if current == end:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return current_cost, path[::-1]

        if current in visited:
            continue

        visited.add(current)

        for next_node in graph.can_see(current):
            if next_node not in visited:
                new_cost = current_cost + euclid_distance(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    frontier.put((new_cost, next_node))
                    parent[next_node] = current

    return float('inf'), []
def search(graph, start, goal, func):
    closed = set()
    queue = PriorityQueue()
    queue.put((0 + func(graph, start), start))
    if start not in closed:
        closed.add(start)
    while not queue.empty():
        cost, node = queue.get()
        if node == goal:
            return node
        for i in graph.can_see(node):
            new_cost = node.g + euclid_distance(node, i)
            if i not in closed or new_cost < i.g:
                closed.add(i)
                i.g = new_cost
                i.pre = node
                new_cost = func(graph, i)
                queue.put((new_cost, i))
    return node
a_star = lambda graph, i: i.g + graph.h(i)
greedy = lambda graph, i: graph.h(i)

def main():
    n_polygon = 0
    poly_list = list(list())
    x = list()
    y = list()
             
    with open('Input.txt', 'r') as f:
        line = f.readline()
        line = line.strip()
        line = line.split()
        line = list(map(int, line))
        n_polygon = line[0]
        start = Point(line[1], line[2])
        goal = Point(line[3], line[4])
        poly_list.append([start])
        for line in f:
            point_list = list()
            line = line.split()
            n_vertex = int(line[0])
            for j in range(0, 2*n_vertex, 2):
                point_list.append(Point(int(line[j + 1]), int(line[j + 2])))
            poly_list.append(point_list[:])
        poly_list.append([goal])
        graph = Graph(poly_list)
        graph.heuristic = {point: point.heuristic(goal) for point in graph.get_points()}

        a = search(graph, start, goal, greedy)

        result = list()

        print("Chọn thuật toán:")
        print("1. A*")
        print("2. Greedy")
        print("3. BFS")
        print("4. DFS")
        print("5. UCS")
        thuat_toan = int(input("Nhập lựa chọn của bạn: "))

        # Khởi chạy thuật toán
        result = []
        if thuat_toan == 1:
            algorithm_name = 'A*'
            a = search(graph, start, goal, a_star)
            while a:
                result.append(a)
                a = a.pre
            result.reverse()
        elif thuat_toan == 2:
            algorithm_name = 'Greedy'
            a = search(graph, start, goal, greedy)
            while a:
                result.append(a)
                a = a.pre
            result.reverse()
        elif thuat_toan == 3:
            algorithm_name = 'BFS'
            result = BFS(graph, start, goal)
        elif thuat_toan == 4:
            algorithm_name = 'DFS'
            result = DFS(graph, start, goal)
        elif thuat_toan == 5:
            algorithm_name = 'UCS'
            _, result = UCS(graph, start, goal)
        else:
            print("Lựa chọn không hợp lệ!")
            return

        # In kết quả
        plt.figure()
        plt.title(f"Đường đi của thuật toán {algorithm_name}")
        plt.plot([start.x], [start.y], 'bo', label="Start")
        plt.plot([goal.x], [goal.y], 'go', label="Goal")
        print("Path found:")
        print(" -> ".join(map(str, result)))
        result.reverse()
        print_res = [[point, point.polygon_id] for point in result]
        print(*print_res, sep=' ->')


        for point in graph.get_points():
            x.append(point.x)
            y.append(point.y)
        plt.plot(x, y, 'ro')
        plt.plot(start.x, start.y, 'bo') 
        plt.plot(goal.x, goal.y, 'go') # Node Goal sẽ được tô màu xanh lục cho dễ nhìn
        for i in range(1, len(poly_list) - 1):
            coord = list()
            for point in poly_list[i]:
                coord.append([point.x, point.y])
            coord.append(coord[0])
            xs, ys = zip(*coord)
            plt.plot(xs, ys)
        x = list()
        y = list()

        for point in result:
            x.append(point.x)
            y.append(point.y)
        if x == None:
            plt.plot(label= "No path") 

        # Chú thích kí hiệu đương đi cho người đọc dễ hiểu
        plt.plot(x, y, 'b', linewidth=2.0, label="Path")
        plt.legend()
        plt.show()

if __name__ == "__main__":
    main()



