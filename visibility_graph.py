import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
from shapely.geometry import Polygon, Point, asPoint, asMultiPoint, LineString, asMultiLineString, MultiPolygon, MultiLineString
from shapely.ops import unary_union

np.random.seed(2)


def generate_obstacle(center, radius, min_num_vertices, num_cuts):
    # generate a convex hull with at least min_num_vertices vertices
    num_vertices = 0
    hull = None
    num_gen_points = min_num_vertices * 10
    while num_vertices < min_num_vertices:
        rand_points = radius * \
            np.random.normal(0., 1., size=(num_gen_points, 2)) + center
        hull = asMultiPoint(rand_points).convex_hull
        num_vertices = len(hull.exterior.coords)
        num_gen_points *= 2

    # generate cuts (may rarely lead to invalid polygon)
    obstacle = generate_cuts(hull, num_cuts, radius)
    while not obstacle.is_valid:
        obstacle = generate_cuts(hull, num_cuts, radius)

    return obstacle


def generate_cuts(polygon, num_cuts, cut_radius):
    cut_radius = cut_radius
    for i in range(num_cuts):
        vertices = polygon.exterior.coords
        idx = np.random.choice(len(vertices) - 1)
        v1 = vertices[idx]
        v2 = vertices[idx + 1]
        midpoint = asPoint(0.5 * (np.array(v2) + np.array(v1)))
        cut_point = sample_point(midpoint, cut_radius)
        while not polygon.contains(cut_point):
            cut_point = sample_point(midpoint, cut_radius)
        new_vertices = vertices[:idx + 1] + [cut_point] + vertices[idx + 1:]
        polygon = Polygon(new_vertices)
    return polygon


def generate_obstacles(num_obstacles, min_num_vertices):
    obstacles = []
    for i in range(num_obstacles):
        vertices = []
        center = asPoint(np.random.uniform(-10., 10., size=(2,)))
        radius = np.random.uniform(1., 2.)
        num_cuts = 3 if toss_coin() else 0
        obstacles.append(generate_obstacle(
            center, radius, min_num_vertices, num_cuts))
    return obstacles


def sample_point(point, radius):
    return asPoint(radius * np.random.uniform(-1., 1., size=(2,)) + np.array(point))


def toss_coin():
    return np.random.uniform(0., 1.) > 0.5


class Node:
    def __init__(self, point, neighbors=[]):
        self.point = point
        self.neighbors = neighbors


def create_visibility_graph(obstacles):
    nodes = [[Node(vertex) for vertex in obstacle.exterior.coords] for obstacle in obstacles]
    edges = []
    obstacle_union = unary_union(obstacles)
    for io, obstacle in enumerate(obstacles):
        coords = obstacle.exterior.coords
        for iv, vertex in enumerate(coords):
            curr_node = nodes[io][iv]
            prev_node = nodes[io][(iv - 1) % len(coords)]
            next_node = nodes[io][(iv + 1) % len(coords)]
            curr_node.neighbors += [prev_node, next_node]
            edges.append((curr_node, next_node))
            for jo in range(io + 1, len(obstacles)):
                other_obstacle = obstacles[jo]
                other_coords = other_obstacle.exterior.coords
                for jv, other_vertex in enumerate(other_coords):
                    line = LineString([vertex, other_vertex])
                    if line.touches(obstacle_union):
                        other_node = nodes[jo][jv]
                        curr_node.neighbors.append(other_node)
                        other_node.neighbors.append(curr_node)
                        edges.append((curr_node, other_node))
    return nodes, edges


def create_extended_visibility_graph(obstacles):
    # TODO: implementation
    pass


def draw_polygon(polygon, color='blue'):
    plt.gca().add_patch(patches.Polygon(polygon.exterior, closed=False, color=color))


def draw_line_string(line_string, color='red'):
    x, y = line_string.xy
    plt.plot(x, y, color=color)


def main():
    obstacles = generate_obstacles(num_obstacles=3, min_num_vertices=6)
    nodes, edges = create_visibility_graph(obstacles)

    for obstacle in obstacles:
        draw_polygon(obstacle)
    for edge in edges:
        draw_line_string(LineString([edge[0].point, edge[1].point]))

    plt.xlim(-20., 20.)
    plt.ylim(-20., 20.)
    plt.show()


if __name__ == '__main__':
    main()
