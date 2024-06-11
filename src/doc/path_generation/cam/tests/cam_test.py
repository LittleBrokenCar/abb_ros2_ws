import cam
from ament_index_python.packages import get_package_share_directory
# if __name__ == "__main__":
#     """通过测试"""
# vtkAdaptor = VtkAdaptor()
# vtkAdaptor.setBackgroundColor(0.95, 0.95, 0.95)
# vtkAdaptor.drawAxes()
# # vtkAdaptor.drawPoint(Point3D(10,10,10)).GetProperty().SetColor(1,0,0)
# # vtkAdaptor.drawPoint(Point3D(50,50,50)).GetProperty().SetColor(0,1,0)
# # polyline = PolyLine()
# # polyline.addPoint(Point3D(1,1,1))
# # polyline.addPoint(Point3D(50, 2, 10))
# # polyline.addPoint(Point3D(20, 10, 30))
# # polyline.addPoint(Point3D(50, 80, 55))
# # polylineActor = vtkAdaptor.drawPolyline(polyline)
# # polylineActor.GetProperty().SetColor(0.1,0.7,0.7)
# # polylineActor.GetProperty().SetLineWidth(2)
#
# stlActor = vtkAdaptor.drawStlModel("done.stl")
# stlActor2 = vtkAdaptor.drawStlModel("done.stl")
# sx, sy, sz = stlActor2.GetScale()
# x, y, z = stlActor2.GetPosition()
# sx, sy, sz = sx / 2.0, sy / 2.0, sz / 1.1
# x += 125.0
# stlActor2.SetPosition(x, y, z)
# stlActor2.SetScale(sx, sy, sz)
# stlActor2.RotateY(-45.0)
#
# vtkAdaptor.display()


# if __name__ == "__main__":
#     """通过测试"""
#     """test intersectStl_brutal() and linkSeg_brutal()"""
#     vtkAdaptor = VtkAdaptor()
#     vtkAdaptor.drawAxes()
#
#     vtkStlReader = vtk.vtkSTLReader()
#     vtkStlReader.SetFileName('surface.stl')
#
#     vtkAdaptor.drawPdSrc(vtkStlReader).GetProperty().SetOpacity(0.5)
#     stlModel = StlModel()
#     stlModel.extraFromVtkStlReader(vtkStlReader)
#
#     layers = intersectStl_brutal(stlModel, 1, zBase=6)
#     for layer in layers:
#         for seg in layer.segments:
#             segActor = vtkAdaptor.drawSegment(seg)
#             segActor.GetProperty().SetLineWidth(2)
#         # layer.contours = linkSeg_brutal(layer.segments)  # 此平面交界线无法闭合 程序死循环
#
#         # for contour in layer.contours:
#         #     polyActor = vtkAdaptor.drawPolyline(contour)
#         #     polyActor.GetProperty().SetLineWidth(2)
#     vtkAdaptor.display()

# if __name__ == "__main__":
#     """通过测试"""
#     P = Point3D(10, 0, 10)
#     N = Point3D(10,0,-10)
#     A = Point3D(-10, -10, 0)
#     B = Point3D(10, 20, 0)
#     C = Point3D(30, -10, 0)
#     ABC = Triangle(A, B, C, Vector3D(0, 0, 1))
#
#     vtkAdaptor = VtkAdaptor()
#     vtkAdaptor.setBackgroundColor(0.95, 0.95, 0.95)
#     vtkAdaptor.drawAxes()
#     vtkAdaptor.drawPoint(P).GetProperty().SetColor(1, 0, 0)
#     vtkAdaptor.drawSegment(Segment(P,N)).GetProperty().SetColor(1, 0, 0)
#     vtkAdaptor.drawTriangle(ABC).GetProperty().SetColor(1, 0, 0)
#
#     Q = intersect(Line(P,Vector3D(0,0,1)), ABC)
#     if Q:
#         vtkAdaptor.drawPoint(Q).GetProperty().SetColor(1, 1, 0)
#     vtkAdaptor.display()

# Demo1.0
# if __name__ == "__main__":
#     points = read_2D_Gcode(offset_x=190, offset_y=130, scale=5e-4)
#
#     vtkAdaptor = VtkAdaptor()
#     vtkAdaptor.drawAxes()
#     vtkStlReader = vtk.vtkSTLReader()
#     vtkAdaptor.setBackgroundColor(0.95, 0.95, 0.95)
#     vtkStlReader.SetFileName('bread_processed.stl')
#     vtkAdaptor.drawPdSrc(vtkStlReader).GetProperty().SetOpacity(0.5)
#     stlModel = StlModel()
#     stlModel.extraFromVtkStlReader(vtkStlReader)
#     polyline = PolyLine()
#     polyline_result = PolyLine()
#     for point in points:
#         polyline.addPoint(point)
#     polylineActorOriginPoints = vtkAdaptor.drawPolyline(polyline)
#
#     inter_points,nor_vectors = projection_point2stlModel(points, stlModel)
#     for i in range(len(inter_points)):
#         polyline_result.addPoint(inter_points[i])
#         print("X{} Y{} Z{}".format(inter_points[i].x, inter_points[i].y, inter_points[i].z))
#     polylineActorInterPoints = vtkAdaptor.drawPolyline(polyline_result)
#     for i in range(len(inter_points)):
#         vtkAdaptor.drawNorVector(inter_points[i], nor_vectors[i])
#         print("x{} y{} z{}".format(nor_vectors[i].dx, nor_vectors[i].dy, nor_vectors[i].dz))
#     vtkAdaptor.display()

# Demo2.0
if __name__ == "__main__":
    gcode_filename="gcode_TriangleOnComplexSurface_0.0005.txt"
    points = cam.read_2D_Gcode(Gcode_filepath=get_package_share_directory("abb_ros2_moveit_config")+"/config/plan/"+gcode_filename)

    vtkAdaptor = cam.VtkAdaptor()
    vtkAdaptor.drawAxes(length=1, cylinderRadius=0.005, coneRadius=0.2)
    vtkStlReader = cam.vtk.vtkSTLReader()
    vtkAdaptor.setBackgroundColor(0.95, 0.95, 0.95)
    vtkStlReader.SetFileName(get_package_share_directory("abb_support")+"/meshes/workspace/visual/substrate.stl")
    vtkAdaptor.drawPdSrc(vtkStlReader).GetProperty().SetOpacity(0.5)
    stlModel = cam.StlModel()
    stlModel.extraFromVtkStlReader(vtkStlReader)
    polyline = cam.PolyLine()
    polyline_result = cam.PolyLine()
    for point in points:
        polyline.addPoint(point)
        # print(point)
    polylineActorOriginPoints = vtkAdaptor.drawPolyline(polyline)

    inter_points,nor_vectors = cam.projection_point2stlModel(points, stlModel, dz=-1.0)
    for i in range(len(inter_points)):
        polyline_result.addPoint(inter_points[i])
        # print("X{} Y{} Z{}".format(inter_points[i].x, inter_points[i].y, inter_points[i].z))
        print("X{} Y{} Z{} x0.0 y0.0 z0.0 w1.0".format(inter_points[i].x, inter_points[i].y, inter_points[i].z))
    polylineActorInterPoints = vtkAdaptor.drawPolyline(polyline_result)
    # for i in range(len(inter_points)):
    #     vtkAdaptor.drawNorVector(inter_points[i], nor_vectors[i])
    #     print("x{} y{} z{}".format(nor_vectors[i].dx, nor_vectors[i].dy, nor_vectors[i].dz))
    vtkAdaptor.display()

