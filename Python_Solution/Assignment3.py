import math

# Function to calculate Euclidean distance
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def merge_sort(lst):
    if len(lst) <= 1:
        return lst

    mid = len(lst) // 2
    left_half = merge_sort(lst[:mid])
    right_half = merge_sort(lst[mid:])

    return merge(left_half, right_half)

def merge(left, right):
    merged = []
    left_index = 0
    right_index = 0

    # Merge smaller elements first
    while left_index < len(left) and right_index < len(right):
        if left[left_index][0] > right[right_index][0]:
            merged.append(left[left_index])
            left_index += 1

        else:
            merged.append(right[right_index])
            right_index += 1

    # If there are remaining elements in the left or right half, append them to the result
    while left_index < len(left):
        merged.append(left[left_index])
        left_index += 1

    while right_index < len(right):
        merged.append(right[right_index])
        right_index += 1

    return merged

# Find the shortest path using Dijkstra's algorithm
def shortest_path(graph, start, end):
    queue = [(0, start, [])]
    seen = set()

    while queue:
        queue = merge_sort(queue) # Sort the queue in descending order
        (cost, node, path) = queue.pop() # Pop the last element (smallest cost)

        if node not in seen:
            seen.add(node)
            path = path + [node]

            if node == end:
                return cost, path

            for (next_node, weight) in graph[node]:
                if next_node not in seen:
                    queue.append((cost + weight, next_node, path))

# Find the longest path using Depth-First Search (DFS)
def longest_path(graph, start, end, visited, path, max_path, max_length, weight):
    visited[start] = True
    path.append(start)

    if start == end:
        if weight > max_length[0]:
            max_length[0] = weight
            max_path[0] = path.copy()

    else:
        for node, w in graph[start]:
            if visited[node] == False:
                longest_path(graph, node, end, visited, path, max_path, max_length, weight + w)

    path.pop()
    visited[start] = False

def main():
    filename = input("Enter file name: ")

    with open(filename, 'r') as file:
        lines = file.readlines()
        nVertices_nEdges = lines[0].split()
        nVertices = int(nVertices_nEdges[0])
        nEdges = int(nVertices_nEdges[1])
        vertices = {}
        edges = []
        graph = {}
        for i in range(1, nVertices + 1):
            graph[i] = []

        for i in range(1, nVertices + 1):
            k_x_y = lines[i].split()
            k = int(k_x_y[0]) # k is the vertex label
            x = float(k_x_y[1]) # x is the x coordinate of the vertex
            y = float(k_x_y[2]) # y is the y coordinate of the vertex
            vertices[k] = (x, y)

        for i in range(nVertices+1, nVertices + nEdges + 1):
            i_j_w = lines[i].split()
            i = int(i_j_w[0]) # i is the labels of the start vertex of the edge
            j = int(i_j_w[1]) # j is the labels of the end vertex of the edge
            w = float(i_j_w[2]) # w is the weight of the edge
            edges.append((i, j, w))
            graph[i].append((j, w))

        start_end = lines[-1].split()
        start_vertex_label = int(start_end[0])
        goal_vertex_label = int(start_end[1])

    print("The number of vertices in the graph:", nVertices)
    print("The number of edges in the graph:", nEdges)
    print("The start vertices:", start_vertex_label)
    print("The end vertices:", goal_vertex_label)
    print("The Euclidean distance between the start and the goal vertices:", euclidean_distance(*vertices[start_vertex_label], *vertices[goal_vertex_label]))

    shortest_path_cost, shortest_path_vertices = shortest_path(graph, start_vertex_label, goal_vertex_label)

    print("The vertices on the shortest path:", shortest_path_vertices)
    print("The length of the shortest path:", shortest_path_cost)

    visited = {}
    for i in range(1, nVertices + 1):
        visited[i] = False
    path, max_path  = [], [[]]
    max_length = [0]

    longest_path(graph, start_vertex_label, goal_vertex_label, visited, path, max_path, max_length, 0)

    print("The vertices on the longest path:", max_path[0])
    print("The length of the longest path:", max_length[0])

main()