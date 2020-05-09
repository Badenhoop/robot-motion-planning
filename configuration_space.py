from sympy.geometry import Polygon, Segment2D, Point2D
from matplotlib import pyplot as plt
from matplotlib import patches
import numpy as np


def rectangle(x, y, width, height):
    return Polygon((x, y), (x + width, y), (x + width, y + height), (x, y + height))


class Joint:
    def __init__(self, theta_initial=0., theta_min=float('-inf'), theta_max=float('inf')):
        self.theta_initial = theta_initial
        self.theta_min = theta_min
        self.theta_max = theta_max


class Link:
    def __init__(self, joint, length):
        self.joint = joint
        self.length = length


class Robot:
    def __init__(self, links, pos):
        self.links = links
        self.pos = pos
        self.config = [link.joint.theta_initial for link in links]

    def check_collision(self, obstacles):
        result = False
        joint_pos = self.pos
        for i, link in enumerate(self.links):
            theta = self.config[i]
            link_end_point = Point2D(
                joint_pos.x + link.length * np.cos(theta),
                joint_pos.y + link.length * np.sin(theta))
            link_segment = Segment2D(joint_pos, link_end_point)
            for obstacle in obstacles:
                if link_segment.intersection(obstacle):
                    return True
            joint_pos = link_end_point
        return result


def collision_grid(robot, obstacles, resolution=0.2 * np.pi):
    config_spaces = []
    grid_shape = []
    for i, link in enumerate(robot.links):
        theta_min = max(link.joint.theta_min, 0.)
        theta_max = min(link.joint.theta_max, 2. * np.pi)
        config_space = np.arange(theta_min, theta_max, resolution)
        config_spaces.append(config_space)
        grid_shape.append(len(config_space))

    grid = np.zeros(grid_shape)
    def check_collision(indices, config):
        robot.config = config
        grid[indices] = robot.check_collision(obstacles)
    iter_spaces(config_spaces, check_collision)
    return grid


def iter_spaces(spaces, f, indices=(), values=[]):
    if len(spaces) == 0:
        f(indices, values)
        return
    curr_space = spaces[0]
    for index, value in enumerate(curr_space):
        iter_spaces(spaces[1:], f, indices + (index,), values + [value])


def draw_robot(robot):
    joint_pos = robot.pos
    for i, link in enumerate(robot.links):
        theta = robot.config[i]
        x_curr = joint_pos.x
        y_curr = joint_pos.y
        x_next = x_curr + link.length * np.cos(theta)
        y_next = y_curr + link.length * np.sin(theta)
        plt.plot([x_curr, x_next], [y_curr, y_next], color=(0., 0., 1.))
        plt.plot([x_curr, x_next], [y_curr, y_next], 'o', color=(0., 0., 1.))
        joint_pos = Point2D(x_next, y_next)


def draw_polygon(polygon):
    plt.gca().add_patch(patches.Polygon(polygon.vertices, closed=False))


def main():
    # setup scene
    obstacles = [rectangle(-3., 2., 3., 2.), rectangle(2., 3., 2., 3.)]
    joints = [
        Joint(np.deg2rad(45.), np.deg2rad(0.), np.deg2rad(180.)),
        Joint(np.deg2rad(120.)),
    ]
    links = [
        Link(joints[0], 3.),
        Link(joints[1], 2.),
    ]
    robot = Robot(links, Point2D(0., 0.))

    fig, axes = plt.subplots(2)

    # draw scene
    plt.sca(axes[0])
    for obstacle in obstacles:
        draw_polygon(obstacle)
    draw_robot(robot)

    # draw collision map
    plt.sca(axes[1])
    grid = collision_grid(robot, obstacles, 0.1 * np.pi)
    plt.imshow(grid)

    plt.show()


if __name__ == '__main__':
    main()
