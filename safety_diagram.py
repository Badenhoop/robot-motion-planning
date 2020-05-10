import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
from shapely.geometry import *
from shapely.ops import nearest_points
from descartes.patch import PolygonPatch


def safety_diagram(bounds, obstacles, resolution):
    x_min, y_min, x_max, y_max = bounds
    cols = int((x_max - x_min) / resolution) + 1
    rows = int((y_max - y_min) / resolution) + 1
    distances = np.zeros((rows, cols))
    for r in range(rows):
        for c in range(cols):
            x = x_min + resolution / 2. + c * resolution
            y = y_min + resolution / 2. + r * resolution
            point = Point(x, y)
            line = LineString(nearest_points(point, obstacles))
            distance = line.length
            distances[r, c] = distance
    return distances


def draw_polygon(polygon, color='blue'):
    patch = PolygonPatch(polygon, facecolor=color,
                         edgecolor=color, alpha=0.5, zorder=2)
    plt.gca().add_patch(patch)


def main():
    ext = [(-12., -12.), (-12., 12.), (12., 12.), (12., -12.)]
    int = [(-10., -10.), (10., -10.), (10., 10.), (-10., 10.)]
    boundary = Polygon(ext, [int])

    obstacles = MultiPolygon([
        boundary,
        Polygon([(-2., -2.), (-2., 1.), (-1., 1), (-1., -2.)]),
        Polygon([(3., 3.), (6., 3.), (6., 5), (3., 5.)]),
    ])

    diag = safety_diagram(bounds=(-10., -10., 10., 10.),
                           obstacles=obstacles, resolution=0.1)

    fig, axes = plt.subplots(2)

    plt.sca(axes[0])

    for obstacle in obstacles:
        draw_polygon(obstacle)

    plt.xlim(-15., 15.)
    plt.ylim(-15., 15.)
    plt.gca().set_aspect('equal')

    plt.sca(axes[1])
    plt.imshow(diag)
    plt.gca().set_aspect('equal')

    plt.show()


if __name__ == '__main__':
    main()
