import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
import shapely
from shapely.geometry import *
from shapely.ops import unary_union

np.random.seed(5)


def generate_obstacles(
        num_obstacles,
        bounds = (-10., -10., 10., 10.),
        min_radius = 2.,
        max_radius = 10.,
        num_vertices_range = list(range(6, 15))):
    obstacles = []
    x_min, y_min, x_max, y_max = bounds
    for i in range(num_obstacles):
        vertices = []
        x_center = np.random.uniform(x_min, x_max)
        y_center = np.random.uniform(y_min, y_max)
        center = Point(x_center, y_center)
        radius = np.random.uniform(min_radius, max_radius)
        num_vertices = np.random.choice(num_vertices_range)
        obstacles.append(generate_obstacle(
            center, radius, num_vertices))
    union = unary_union(obstacles)
    return union if isinstance(union, MultiPolygon) else MultiPolygon([union])


def generate_obstacle(center, radius, num_vertices):
    angle_increment = 2. * np.pi / num_vertices
    angles = angle_increment * np.arange(num_vertices)
    radi = radius * np.random.uniform(0.1, 1.0, size=(num_vertices,))
    vertices = np.zeros((num_vertices, 2))
    vertices[:, 0] = radi * np.cos(angles)
    vertices[:, 1] = radi * np.sin(angles)
    vertices += center
    return asPolygon(vertices)


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

    union = unary_union(entities) 
    return union if isinstance(union, MultiLineString) else MultiLineString([union])


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
            combined = MultiPolygon(
                [obstacle.convex_hull, other_obstacle.convex_hull])
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
            other_enclosing_points = enclosing_lines.intersection(
                other_obstacle)
            entities.append(seperating_lines(
                obstacle, obstacles, other_enclosing_points))
            entities.append(seperating_lines(
                other_obstacle, obstacles, enclosing_points))

    union = unary_union(entities) 
    return union if isinstance(union, MultiLineString) else MultiLineString([union])


def seperating_lines(obstacle, obstacles, enclosing_points):
    lines = []
    for vertex in obstacle.boundary.coords:
        line1 = LineString([vertex, enclosing_points[0]])
        line2 = LineString([vertex, enclosing_points[1]])
        if line1.touches(obstacles) and line2.touches(obstacles):
            lines += [line1, line2]
    return MultiLineString(lines)


def draw_polygon(polygon, color='blue'):
    plt.gca().add_patch(patches.Polygon(polygon.exterior, closed=False, color=color))


def draw_line_string(line_string, color='red'):
    x, y = line_string.xy
    plt.plot(x, y, color=color)


def main():
    obstacles = generate_obstacles(num_obstacles=3)

    fig, axes = plt.subplots(2)

    lines = create_visibility_graph(obstacles)
    plt.sca(axes[0])
    for obstacle in obstacles:
        draw_polygon(obstacle)
    for line in lines:
        draw_line_string(line)

    plt.title('visibility graph')
    plt.xlim(-15., 15.)
    plt.ylim(-15., 15.)
    plt.gca().set_aspect('equal')

    lines = create_simplified_visibility_graph(obstacles)
    plt.sca(axes[1])
    for obstacle in obstacles:
        draw_polygon(obstacle)
    for line in lines:
        draw_line_string(line)

    plt.title('simplified visibility graph')
    plt.xlim(-15., 15.)
    plt.ylim(-15., 15.)
    plt.gca().set_aspect('equal')

    plt.show()


if __name__ == '__main__':
    main()
