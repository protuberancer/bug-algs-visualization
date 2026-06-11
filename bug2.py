# Определяем окружение
boundary = [[0, 0], [10, 0], [10, 10], [0, 10]]
# Массив четырёхугольников: первая точка - левая нижняя, последующие против часовой, все координаты больше 0
obstacles = [
    [[0, 3], [2, 3], [2, 4], [0, 4]],
    [[2, 1], [7, 1], [7, 7], [2, 4]],
    [[7.5, 0], [8, 0], [8, 8], [7.5, 8]],
    # Границы области
    [[0, 0], [10, 0], [10, 0.25], [0, 0.25]],
    [[0, 0], [0.25, 0], [0.25, 10], [0, 10]],
    [[0, 9.75], [10, 9.75], [10, 10], [0, 10]],
    [[9.75, 0], [10, 0], [10, 10], [9.75, 10]],
]

stepSize = 0.1
robotSize = 0.25
rotation = -1  # 1: вправо, -1: влево

# Стартовая и целевая позиции
start = [1, 1]
goal = [9, 9]

# Визуализация
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import numpy as np
from shapely.geometry import Point, Polygon, LineString


def find_nearest_side_index(point, path):
    # Найти ближайшую сторону многоугольника до заданной точки
    min_distance = float('inf')
    nearest_side_index = 0
    for i in range(len(path.vertices)):
        start_vertex = path.vertices[i]
        end_vertex = path.vertices[(i + 1) % len(path.vertices)]
        segment = np.array([start_vertex, end_vertex])
        distance = np.linalg.norm(np.cross(segment[1] - segment[0], segment[0] - point)) / np.linalg.norm(segment[1] - segment[0])

        if distance < min_distance:
            min_distance = distance
            nearest_side_index = i
    return nearest_side_index

def find_side_vector(side_index, path):

    # Получить вершины ближайшей стороны
    side_vertices = np.array([path.vertices[side_index],
                              path.vertices[(side_index + 1) % len(path.vertices)]])

    # Найти вектор, представляющий направление ближайшей стороны многоугольника
    side_vector = side_vertices[1] - side_vertices[0]

    # Направление вдоль стенки (по часовой стрелке)
    direction_vector = np.array([side_vector[0], side_vector[1]])
    direction_vector /= np.linalg.norm(direction_vector)

    return direction_vector

def find_vector_to_nearest_side(point, path):
    # Получить индекс ближайшей стороны
    nearest_side_index = find_nearest_side_index(point, path)

    direction_vector = find_side_vector(nearest_side_index, path)

    return direction_vector

def at_vertex(robot_pos, obstacle):
    # Найти ближайшую сторону к текущему положению робота
    nearest_side_index = find_nearest_side_index(robot_pos, obstacle)

    # Получить вершины ближайшей стороны
    side_vertices = np.array([obstacle.vertices[nearest_side_index],
                              obstacle.vertices[(nearest_side_index + 1) % len(obstacle.vertices)]])

    # Найти расстояние до вершины стороны
    distance_to_vertex = min(np.linalg.norm(side_vertices[0] - robot_pos),
                             np.linalg.norm(side_vertices[1] - robot_pos))

    # Если расстояние до вершины меньше радиуса робота, считаем, что робот на углу
    return distance_to_vertex < robotSize

fig, ax = plt.subplots()
for obstacle in obstacles:
    ax.add_patch(PathPatch(Path(obstacle), color=[0.7, 0.7, 0.7]))

# Define Axes
x_points = [start[0], goal[0]]
y_points = [start[1], goal[1]]

# Plot a graph
plt.plot(x_points, y_points, linestyle='dashed')

ax.plot(*zip(*boundary + [boundary[0]]), 'k-')  # Граница
ax.plot(*start, 'ro', markersize=10)
ax.plot(*goal, 'go', markersize=10)
ax.axis('equal')
ax.grid(True)
ax.set_title('Bug2')

# Параметры
robotPos = start
followBoundary = False
currentObstacle = None
direction = None
current_side_index = None
mline = LineString([start, goal])

# Увеличиваем объём препятствий для робота, чтобы робот не приближался к препятствиям ближе чем на расстояние robotSize
#for obstacle in obstacles:
#    obstacle[0] = [obstacle[0][0] - robotSize, obstacle[0][1] - robotSize]
#    obstacle[1] = [obstacle[1][0] + robotSize, obstacle[1][1] - robotSize]
#    obstacle[2] = [obstacle[2][0] + robotSize, obstacle[2][1] + robotSize]
#    obstacle[3] = [obstacle[3][0] - robotSize, obstacle[3][1] + robotSize]


while abs(robotPos[0] - goal[0]) > stepSize or abs(robotPos[1] - goal[1]) > stepSize:
    if not followBoundary:
        # Движение напрямую к цели
        direction = [(goal[0] - robotPos[0]) / np.linalg.norm([goal[0] - robotPos[0], goal[1] - robotPos[1]]),
                     (goal[1] - robotPos[1]) / np.linalg.norm([goal[0] - robotPos[0], goal[1] - robotPos[1]])]
        nextPos = [robotPos[0] + stepSize * direction[0], robotPos[1] + stepSize * direction[1]]

        # Проверяем столкновение с препятствиями
        for obstacle in obstacles:
            if Path(obstacle).contains_point(nextPos):
                followBoundary = True
                currentObstacle = Path(obstacle)
                direction = rotation * find_vector_to_nearest_side(robotPos, currentObstacle)
                current_side_index = find_nearest_side_index(robotPos, currentObstacle)
                print('Obstacle hit! Following the boundary.')
                nextPos = [robotPos[0] + stepSize * direction[0], robotPos[1] + stepSize * direction[1]]

    else:
        # Движение вдоль границы препятствия
        if mline.distance(Point(robotPos)) < stepSize:
            followBoundary = False

        else:
            if at_vertex(robotPos, currentObstacle):  # Проверяем, находится ли робот на вершине
                print('Robot reached vertex. Rotating.')
                hit = False
                current_obstacle_polygon = Polygon(currentObstacle.vertices)
                while True:
                    nextPos = [robotPos[0] + stepSize * direction[0], robotPos[1] + stepSize * direction[1]]
                    robot_point = Point(nextPos)
                    distance_to_obstacle = robot_point.distance(current_obstacle_polygon)
                    # Проверяем снова столкновение с препятствием
                    for obstacle in obstacles:
                        if Path(obstacle).contains_point(nextPos):
                            currentObstacle = Path(obstacle)
                            current_side_index = find_nearest_side_index(robotPos, currentObstacle)
                            direction = rotation * find_vector_to_nearest_side(robotPos, currentObstacle)
                            print('Obstacle hit! Following the boundary.')
                            hit = True

                    if hit: break

                    if mline.distance(Point(robotPos)) < stepSize:
                        followBoundary = False
                        break
                    if distance_to_obstacle > 1.4 * stepSize: break
                    robotPos = nextPos

                    ax.plot(robotPos[0], robotPos[1], 'b.')
                    plt.pause(0.05)

                if not hit:
                    # Находим индекс следующей стороны препятствия
                    next_side_index = (current_side_index + rotation) % len(currentObstacle.vertices)

                    direction = rotation * find_side_vector(next_side_index, currentObstacle)

                    side_vertices = np.array([currentObstacle.vertices[current_side_index],
                                              currentObstacle.vertices[(current_side_index + 1) % len(currentObstacle.vertices)]])

                    while True:
                        nextPos = [robotPos[0] + stepSize * direction[0], robotPos[1] + stepSize * direction[1]]
                        # Проверяем снова столкновение с препятствием
                        for obstacle in obstacles:
                            if Path(obstacle).contains_point(nextPos):
                                currentObstacle = Path(obstacle)
                                current_side_index = find_nearest_side_index(robotPos, currentObstacle)
                                direction = rotation * find_vector_to_nearest_side(robotPos, currentObstacle)
                                print('Obstacle hit! Following the boundary.')
                                hit = True
                        if hit: break
                        if mline.distance(Point(robotPos)) < stepSize:
                            followBoundary = False
                            break
                        # Найти расстояние до вершины стороны
                        distance_to_vertex = min(np.linalg.norm(side_vertices[0] - robotPos),
                                                 np.linalg.norm(side_vertices[1] - robotPos))
                        if distance_to_vertex > robotSize: break

                        robotPos = nextPos

                        ax.plot(robotPos[0], robotPos[1], 'b.')
                        plt.pause(0.05)

                    current_side_index = next_side_index

            if followBoundary:
                nextPos = [robotPos[0] + stepSize * direction[0], robotPos[1] + stepSize * direction[1]]

        # Проверяем снова столкновение с препятствием
        for obstacle in obstacles:
            if Path(obstacle).contains_point(nextPos):
                if mline.distance(Point(robotPos)) < stepSize:
                    followBoundary = False
                    break
                followBoundary = True
                currentObstacle = Path(obstacle)
                current_side_index = find_nearest_side_index(robotPos, currentObstacle)
                direction = rotation * find_vector_to_nearest_side(robotPos, currentObstacle)
                print('Obstacle hit! Following the boundary.')
                nextPos = [robotPos[0] + stepSize * direction[0], robotPos[1] + stepSize * direction[1]]

    robotPos = nextPos
    ax.plot(robotPos[0], robotPos[1], 'b.')
    plt.pause(0.01)

print('Goal reached!')
plt.show()
