import numpy as np


def transform_vector2quaternion(nor_vec_filepath='pose.txt', initial_pose=[0, 0, 1]):
    try:
        with open(nor_vec_filepath) as f:
            const_initial_pose = np.array(initial_pose) / np.linalg.norm(np.array(initial_pose))
            rows = f.readlines()
            for row in rows:
                x = float(row.split(' ')[0][1:])
                y = float(row.split(' ')[1][1:])
                z = float(row.split(' ')[2][1:])
                np_target_pose = np.array([x, y, z])
                double_theta = np.arccos(np.dot(const_initial_pose, np_target_pose) / np.linalg.norm(np_target_pose))
                np_rotation_axis = np.cross(const_initial_pose, np_target_pose)
                np_rotation_axis = np_rotation_axis / np.linalg.norm(np_rotation_axis)
                np_quaternion = np.array(
                    [np_rotation_axis[0] * np.sin(double_theta / 2), np_rotation_axis[1] * np.sin(double_theta / 2),
                     np_rotation_axis[2] * np.sin(double_theta / 2), np.cos(double_theta / 2)])
                print("x{} y{} z{} w{}".format(np_quaternion[0], np_quaternion[1], np_quaternion[2], np_quaternion[3]))
    except Exception as e:
        print(e)
    return 0


def transform_tcp_point2plan_point(Poe=[[-0.08], [0], [0.1455]], path_filepath="path.txt", quaternion_filepath="quaternion.txt"):
    # Rbe=Rbo Pbe=Rbo*Poe+Pbo
    Poe = np.asarray(Poe)
    try:
        np_tcp_rotation = []
        np_tcp_point = []
        with open(quaternion_filepath) as f:  # Rbe
            rows = f.readlines()
            for row in rows:
                x = float(row.split(' ')[0][1:])
                y = float(row.split(' ')[1][1:])
                z = float(row.split(' ')[2][1:])
                w = float(row.split(' ')[3][1:])
                np_tcp_rotation.append(quaternion_to_rotation_matrix(np.array([x, y, z, w])))
        with open(path_filepath) as f:  # Pbo
            rows = f.readlines()
            for row in rows:
                X = float(row.split(' ')[0][1:])
                Y = float(row.split(' ')[1][1:])
                Z = float(row.split(' ')[2][1:])
                np_tcp_point.append(np.array([[X], [Y], [Z]]))
        if len(np_tcp_rotation) != len(np_tcp_point):
            raise "the lists of points and pose have unequal length"
        else:
            for i in range(len(np_tcp_rotation)):
                Pbe = np.dot(np_tcp_rotation[i], Poe) + np_tcp_point[i]
                # print(np_tcp_point[i])
                print("X{} Y{} Z{}".format(Pbe[0][0],Pbe[1][0],Pbe[2][0]))
    except Exception as e:
        print(e)
    return 0


def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    R = np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])
    return R


def merge_txt(file_1, file_2):
    try:
        with open(file_2) as f:
            rows_2 = f.readlines()
        with open(file_1) as f:
            rows_1 = f.readlines()
        for i in range(len(rows_1)):
            print(rows_1[i].split('\n')[0] + ' ' + rows_2[i].split('\n')[0])
    except Exception as e:
        print(e)
    return 0


if __name__ == "__main__":
    # Bread_Demo
    # transform_vector2quaternion("pose.txt")
    # merge_txt("path.txt", "quaternion.txt")
    # transform_tcp_point2plan_point(L=0.166043, path_filepath="path.txt", nor_vec_filepath="pose.txt")
    # merge_txt("flange_path.txt", "quaternion.txt")

    # Demo2.0
    # transform_vector2quaternion("../Demo_2.0/pose.txt")
    # merge_txt("../Demo_2.0/path.txt", "../Demo_2.0/quaternion.txt")
    # transform_tcp_point2plan_point(L=0.1, path_filepath="../Demo_2.0/path.txt", nor_vec_filepath="../Demo_2.0/pose.txt")

    # Demo 2.0-2
    # transform_vector2quaternion("../Demo_2.0/D2-2/pose.txt")
    # merge_txt("../Demo_2.0/D2-2/path.txt", "../Demo_2.0/D2-2/quaternion.txt")
    # transform_tcp_point2plan_point(L=0.1, path_filepath="../Demo_2.0/D2-2/path.txt", nor_vec_filepath="../Demo_2.0/D2-2/pose.txt")
    # merge_txt("../Demo_2.0/D2-2/flange_path.txt", "../Demo_2.0/D2-2/quaternion.txt")

    # Demo 2.0-Path_TriangleOnConvexModelTest_0.001
    # transform_vector2quaternion("../Demo_2.0/Pose_TriangleOnConvexModelTest_0.001.txt")
    # transform_tcp_point2plan_point(path_filepath="../Demo_2.0/Path_TriangleOnConvexModelTest_0.001.txt",
    #                                quaternion_filepath="../Demo_2.0/Quaternion_TriangleOnConvexModelTest_0.001.txt")
    merge_txt("../Demo_2.0/L6Path_TriangleOnConvexModelTest_0.001.txt",
              "../Demo_2.0/Quaternion_TriangleOnConvexModelTest_0.001.txt")


