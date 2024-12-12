import math

def calculate_distance(point1, point2):
    """
    Calculate the Euclidean distance between two points in 2D space.

    Parameters:
        point1 (list or tuple): Coordinates [x, y] of the first point.
        point2 (list or tuple): Coordinates [x, y] of the second point.

    Returns:
        float: Euclidean distance between the two points.
    """
    return math.sqrt((point2[0] - point1[0]) ** 2 +
                     (point2[1] - point1[1]) ** 2)

def calculate_angle_difference_in_degrees(angle1, angle2):
    """
    Calculate the absolute difference between two angles in degrees.

    Parameters:
        angle1 (float): First angle in degrees.
        angle2 (float): Second angle in degrees.

    Returns:
        float: Absolute difference between the two angles in degrees.
    """
    return abs(angle1 - angle2)

def process_corresponding_points_and_angles(points1, points2):
    """
    Process two arrays of points with x, y coordinates and angles (in degrees),
    calculating the distance and angle difference between corresponding points.

    Parameters:
        points1 (list of lists): First array of points, each containing [x, y, angle_in_degrees].
        points2 (list of lists): Second array of points, each containing [x, y, angle_in_degrees].

    Returns:
        list of tuples: Each tuple contains (distance, angle_difference).
    """
    results = []
    for p1, p2 in zip(points1, points2):
        distance = calculate_distance(p1[:2], p2[:2])
        angle_difference = calculate_angle_difference_in_degrees(p1[2], p2[2])
        results.append((distance, angle_difference))
    return results

# Example usage
if __name__ == "__main__":
    # final pos x y orientation
    points1 = [
        [ 6.77,   -3.76,  -53.66],
        [-0.998,   3.016, 125.62],
        [2.502,   -2.136,  190.667],
        [ 4.743,   0.708,   14.498],
        [ 2.522,  -2.672,   187.136],
        [ -4.203,  0.568,   161.56 ],
        [ -6.019, -3.178,  -151.167 ],
        [ 6.678,  -0.663,   52.35],
        [ -7.140,  0.435 ,  167.274],
        [  2.682, -0.161 ,  216.61 ]

    ]


    #goal pos x y orientation
    points2 = [
        [ 7.239,    -4.139,   -89.09],
        [ -1.197,    3.670 ,   96.625],
        [ 2.752 ,   -2.527,  171.748],
        [ 5.335,    0.813 ,   7.499],
        [ 2.103,    -3.134,  152.061],
        [ -4.690,   0.766 , 153.854],
        [ -6.228,   -3.64 , -160.29 ],
        [ 7.246,   -0.375  ,88.96 ],
        [ -7.702,   0.664 , 130.33 ],
        [ 3.033,   -0.728, 181.88 ]
    ]

    # Process the corresponding points
    results = process_corresponding_points_and_angles(points1, points2)

    # Output the results
    for i, (distance, angle_diff) in enumerate(results):
        print(f"Between point {i} in array1 and point {i} in array2: Distance = {distance:.4f}, Angle Difference = {angle_diff:.4f} degrees")




