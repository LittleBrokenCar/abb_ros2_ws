# Demo2.0的轨迹定义
import math

const_resolution = 0.0005  # m

"""平面金属片的方形0.08*0.05 m 厚0.0005 mm, 打const_square_length的方形 分辨率 const_resolution m"""
# const_square_length = 0.03  # m
# for i in range(int(const_square_length / const_resolution)):
#     print("X{} Y{} Z0.0 x0.0 y0.0 z0.0 w1.0".format(-const_square_length / 2 + const_resolution * i, const_square_length / 2))
# for i in range(int(const_square_length / const_resolution)):
#     print("X{} Y{} Z0.0 x0.0 y0.0 z0.0 w1.0".format(const_square_length / 2, const_square_length / 2 - const_resolution * i))
# for i in range(int(const_square_length / const_resolution)):
#     print("X{} Y{} Z0.0 x0.0 y0.0 z0.0 w1.0".format(const_square_length / 2 - const_resolution * i, -const_square_length / 2))
# for i in range(int(const_square_length / const_resolution)):
#     print("X{} Y{} Z0.0 x0.0 y0.0 z0.0 w1.0".format(-const_square_length / 2, -const_square_length / 2 + const_resolution * i))
# print("X-0.015 Y0.015 Z0.0 x0.0 y0.0 z0.0 w1.0")

"""凸台绘制三角形 打const_square_length的方形 分辨率 const_resolution"""
const_triangle_length = 0.035  # m
const_model_hemisphere_radius = 0.05  # m
for i in range(int(const_triangle_length / const_resolution)):
    X = 0 + const_resolution * i / 2
    Y = const_triangle_length / 1.73205 - const_resolution * 1.73205 * i / 2
    Z = math.sqrt(const_model_hemisphere_radius * const_model_hemisphere_radius - X * X - Y * Y) - 0.005
    print("G0 X{} Y{} F300".format(X, Y))
    # print("X{} Y{} Z-{} x0.0 y0.0 z0.0 w1.0".format(X, Y, Z))
    # print("x{} y{} z{}".format(X/const_model_hemisphere_radius, Y/const_model_hemisphere_radius, (0.005+Z)/const_model_hemisphere_radius))
for i in range(int(const_triangle_length / const_resolution)):
    X = 0.5 * const_triangle_length - const_resolution * i
    Y = -1 / (1.73205 * 2) * const_triangle_length
    Z = math.sqrt(const_model_hemisphere_radius * const_model_hemisphere_radius - X * X - Y * Y) - 0.005
    print("G0 X{} Y{} F300".format(X, Y))
    # print("X{} Y{} Z{}".format(X, Y, Z))
    # print("X{} Y{} Z-{} x0.0 y0.0 z0.0 w1.0".format(X, Y, Z))
    # print("x{} y{} z{}".format(X/const_model_hemisphere_radius, Y/const_model_hemisphere_radius, (0.005+Z)/const_model_hemisphere_radius))
for i in range(int(const_triangle_length / const_resolution)):
    X = -0.5 * const_triangle_length + const_resolution * i / 2
    Y = -1 / (1.73205 * 2) * const_triangle_length + const_resolution * 1.73205 * i / 2
    Z = math.sqrt(const_model_hemisphere_radius * const_model_hemisphere_radius - X * X - Y * Y) - 0.005
    # print("X{} Y{} Z{}".format(X, Y, Z))
    print("G0 X{} Y{} F300".format(X, Y))
    # print("X{} Y{} Z-{} x0.0 y0.0 z0.0 w1.0".format(X, Y, Z))
    # print("x{} y{} z{}".format(X/const_model_hemisphere_radius, Y/const_model_hemisphere_radius, (0.005+Z)/const_model_hemisphere_radius))
# print("X0 Y0.0173205 Z0.041904")

"""凹台绘制三角形 打const_square_length的方形 分辨率 const_resolution"""
# const_triangle_length = 0.03  # m
# const_model_hemisphere_radius = 0.05  # m
# for i in range(int(const_triangle_length / const_resolution)):
#     X = 0 + const_resolution * i / 2
#     Y = const_triangle_length / 1.73205 - const_resolution * 1.73205 * i / 2
#     Z = -math.sqrt(const_model_hemisphere_radius * const_model_hemisphere_radius - X * X - Y * Y) + 0.055
#     # print("G0 X{} Y{} Z{}".format(X, Y, Z))
#     print("x{} y{} z{}".format(-X/const_model_hemisphere_radius, -Y/const_model_hemisphere_radius, (0.055-Z)/const_model_hemisphere_radius))
# for i in range(int(const_triangle_length / const_resolution)):
#     X = 0.5 * const_triangle_length - const_resolution * i
#     Y = -1 / (1.73205 * 2) * const_triangle_length
#     Z = -math.sqrt(const_model_hemisphere_radius * const_model_hemisphere_radius - X * X - Y * Y) + 0.055
#     # print("G0 X{} Y{} Z{}".format(X, Y, Z))
#     print("x{} y{} z{}".format(-X/const_model_hemisphere_radius, -Y/const_model_hemisphere_radius, (0.055-Z)/const_model_hemisphere_radius))
# for i in range(int(const_triangle_length / const_resolution)):
#     X = -0.5 * const_triangle_length + const_resolution * i / 2
#     Y = -1 / (1.73205 * 2) * const_triangle_length + const_resolution * 1.73205 * i / 2
#     Z = -math.sqrt(const_model_hemisphere_radius * const_model_hemisphere_radius - X * X - Y * Y) + 0.055
#     # print("G0 X{} Y{} Z{}".format(X, Y, Z))
#     print("x{} y{} z{}".format(-X/const_model_hemisphere_radius, -Y/const_model_hemisphere_radius, (0.055-Z)/const_model_hemisphere_radius))
# # print("G0 X0 Y0.0173205 Z0.0080958")

"""波浪型基底绘制三角形"""
pass

"""流量传感器图案制作"""
pass

