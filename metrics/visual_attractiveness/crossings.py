from utils.utils import Point
from constants.constants import ZERO_TOLERANCE


# A Python3 program to find if 2 given line segments intersect or not

# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def on_Segment(p, q, r):
    if ((q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
            (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False


def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise

    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.
    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if val > 0:
        # Clockwise orientation
        return 1
    elif val < 0:
        # Counterclockwise orientation
        return 2
    else:
        # Collinear orientation
        return 0


def at_least_two_equal_points(p1, q1, p2, q2):
    points = [p1, q1, p2, q2]
    for i in range(0, len(points) - 1):
        for j in range(i + 1, len(points)):
            point_i = points[i]
            point_j = points[j]
            if abs(point_i.x - point_j.x) < ZERO_TOLERANCE and abs(point_i.y - point_j.y) < ZERO_TOLERANCE:
                return True
    return False


# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(point_p1, point_q1, point_p2, point_q2):
    if at_least_two_equal_points(point_p1, point_q1, point_p2, point_q2):
        return False

    # Find the 4 orientations required for
    # the general and special cases
    orientation_1 = orientation(point_p1, point_q1, point_p2)
    orientation_2 = orientation(point_p1, point_q1, point_q2)
    orientation_3 = orientation(point_p2, point_q2, point_p1)
    orientation_4 = orientation(point_p2, point_q2, point_q1)

    # General case
    if (orientation_1 != orientation_2) and (orientation_3 != orientation_4):
        return True

    # Special Cases

    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if (orientation_1 == 0) and on_Segment(point_p1, point_p2, point_q1):
        return True

    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if (orientation_2 == 0) and on_Segment(point_p1, point_q2, point_q1):
        return True

    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if (orientation_3 == 0) and on_Segment(point_p2, point_p1, point_q2):
        return True

    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if (orientation_4 == 0) and on_Segment(point_p2, point_q1, point_q2):
        return True

    # If none of the cases
    return False


def crossings(solution, points):
    number_of_crossings = 0
    for position_p1 in range(1, len(solution) - 2):
        info_p1 = points[solution[position_p1] - 1]
        info_q1 = points[solution[position_p1 + 1] - 1]
        point_p1 = Point(info_p1['lon'], info_p1['lat'])
        point_q1 = Point(info_q1['lon'], info_q1['lat'])

        for position_p2 in range(position_p1 + 2, len(solution) - 1):
            info_p2 = points[solution[position_p2] - 1]
            info_q2 = points[solution[position_p2 + 1] - 1]
            point_p2 = Point(info_p2['lon'], info_p2['lat'])
            point_q2 = Point(info_q2['lon'], info_q2['lat'])

            if doIntersect(point_p1, point_q1, point_p2, point_q2):
                number_of_crossings += 1

    return number_of_crossings
