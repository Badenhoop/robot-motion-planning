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


def visibility_graph(obstacles):
    segments = []
    obstacle_union = unary_union(obstacles)
    for i, obstacle in enumerate(obstacles):
        segments += polygon2Segments(obstacle)
        # add each segment constructed by vertices of pairwise distinct objects that does not intersect with other obstacles
        for j, other_obstacle in enumerate(obstacles):
            if i == j:
                continue
            for vi in obstacle.exterior.coords:
                for vj in other_obstacle.exterior.coords:
                    segment = LineString([vi, vj])
                    if segment.touches(obstacle_union):
                        segments.append(segment)
    return segments


def polygon2Segments(polygon):
    segments = []
    for i in range(len(polygon.exterior.coords) - 1):
        v1 = polygon.exterior.coords[i]
        v2 = polygon.exterior.coords[i + 1]
        segments.append(LineString([v1, v2]))
    # add segment from last to first vertex
    v_first = polygon.exterior.coords[0]
    v_last = polygon.exterior.coords[-1]
    segments.append(LineString([v_first, v_last]))
    return segments


def draw_polygon(polygon, color='blue'):
    plt.gca().add_patch(patches.Polygon(polygon.exterior, closed=False, color=color))


def draw_line_string(line_string, color='red'):
    x, y = line_string.xy
    plt.plot(x, y, color=color)


def main():
    obstacles = generate_obstacles(num_obstacles=3, min_num_vertices=6)
    edges = visibility_graph(obstacles)

    for obstacle in obstacles:
        draw_polygon(obstacle)
    for edge in edges:
        draw_line_string(edge)

    plt.xlim(-20., 20.)
    plt.ylim(-20., 20.)
    plt.show()


if __name__ == '__main__':
    main()
