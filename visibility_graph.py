import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
from shapely.geometry import Polygon, Point, asPoint, asMultiPoint, LineString, asMultiLineString, MultiPolygon, MultiLineString
from shapely.ops import unary_union

np.random.seed(4)


def generate_obstacle(center, radius, min_num_vertices, num_cuts):
    # generate a convex hull with at least min_num_vertices vertices
    num_vertices = 0
    hull = None
    num_gen_points = min_num_vertices * 10
    while num_vertices < min_num_vertices:
        rand_points = radius * \
            np.random.normal(0., 1., size=(num_gen_points, 2)) + center
        hull = asMultiPoint(rand_points).convex_hull
        num_vertices = len(hull.boundary.coords)
        num_gen_points *= 2

    # generate cuts (may rarely lead to invalid polygon)
    obstacle = generate_cuts(hull, num_cuts, radius)
    while not obstacle.is_valid:
        obstacle = generate_cuts(hull, num_cuts, radius)

    return obstacle


def generate_cuts(polygon, num_cuts, cut_radius):
    cut_radius = cut_radius
    for i in range(num_cuts):
        vertices = polygon.boundary.coords
        idx = np.random.choice(len(vertices) - 1)
        v1 = vertices[idx]
        v2 = vertices[idx + 1]
        centroid = LineString([v1, v2]).centroid
        cut_point = sample_point(centroid, cut_radius)
        while not polygon.contains(cut_point):
            cut_point = sample_point(centroid, cut_radius)
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
    return unary_union(obstacles)


def sample_point(point, radius):
    return asPoint(radius * np.random.uniform(-1., 1., size=(2,)) + np.array(point))


def toss_coin():
    return np.random.uniform(0., 1.) > 0.5


def create_visibility_graph(obstacles):
    entities = []
    N = len(list(obstacles))
    for io, obstacle in enumerate(obstacles):
        entities.append(obstacle.boundary)
        for jo in range(io + 1, N):
            other_obstacle = obstacles[jo]
            for vertex in obstacle.boundary.coords:
                for other_vertex in other_obstacle.boundary.coords:
                    line = LineString([vertex, other_vertex])
                    if line.touches(obstacles):
                        entities.append(line)
    return entities


def create_simplified_visibility_graph(obstacles):
    entities = []
    N = len(list(obstacles))
    for io, obstacle in enumerate(obstacles):
        entities.append(obstacle.boundary)
        for jo in range(io + 1, N):
            other_obstacle = obstacles[jo]
            # Get the enclosing lines
            # -----------------------
            # Get the combined shape of each of the obstacles convex hull.
            combined = MultiPolygon([obstacle.convex_hull, other_obstacle.convex_hull])
            # Get the convex hull of the combined shape.
            hull = combined.convex_hull
            # The enclosing are now the difference between the convex hull and the combined shape. 
            enclosing_lines = hull.boundary.difference(combined.boundary)

            # Add the enclosing lines if there they don't collisde with other obstacles. 
            for line in enclosing_lines:
                if line.touches(obstacles):
                    entities.append(line)
            
            # Get the seperating lines
            # ------------------------
            enclosing_points = enclosing_lines.intersection(obstacle)
            other_enclosing_points = enclosing_lines.intersection(other_obstacle)
            entities.append(seperating_lines(obstacle, obstacles, other_enclosing_points))
            entities.append(seperating_lines(other_obstacle, obstacles, enclosing_points))
    return unary_union(entities)


def seperating_lines(obstacle, obstacles, enclosing_points):
    lines = []
    for vertex in obstacle.boundary.coords:
        line1 = LineString([vertex, enclosing_points[0]])
        line2 = LineString([vertex, enclosing_points[1]])
        if line1.touches(obstacles) and line2.touches(obstacles):
            lines += [line1, line2]
    return MultiLineString(lines)


def draw_polygon(polygon, color='blue'):
    plt.gca().add_patch(patches.Polygon(polygon.boundary, closed=False, color=color))


def draw_line_string(line_string, color='red'):
    x, y = line_string.xy
    plt.plot(x, y, color=color)


def main():
    obstacles = generate_obstacles(num_obstacles=3, min_num_vertices=6)

    fig, axes = plt.subplots(2)

    lines = create_visibility_graph(obstacles)
    plt.sca(axes[0])
    for obstacle in obstacles:
        draw_polygon(obstacle)
    for line in lines:
        draw_line_string(line)

    lines = create_simplified_visibility_graph(obstacles)
    plt.sca(axes[1])
    for obstacle in obstacles:
        draw_polygon(obstacle)
    for line in lines:
        draw_line_string(line)

    plt.show()

if __name__ == '__main__':
    main()
