import vtk
from cam import *


class VtkAdaptor:
    def __init__(self, bgClr=(0.95, 0.95, 0.95)):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(bgClr)
        self.window = vtk.vtkRenderWindow()
        self.window.AddRenderer(self.renderer)
        self.window.SetSize(1000, 1000)
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.window)
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.interactor.Initialize()

    def display(self):
        self.interactor.Start()

    def setBackgroundColor(self, r, g, b):
        return self.renderer.SetBackground(r, g, b)

    def drawAxes(self, length=0.2, shaftType=0, cylinderRadius=0.005, coneRadius=0.1):
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(length, length, length)
        axes.SetShaftType(shaftType)
        axes.SetCylinderRadius(cylinderRadius)
        axes.SetConeRadius(coneRadius)
        axes.SetAxisLabels(0)
        self.renderer.AddActor(axes)
        return axes

    def drawActor(self, actor):
        self.renderer.AddActor(actor)
        return actor

    def drawPdSrc(self, pdSrc):
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(pdSrc.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        return self.drawActor(actor)

    def drawStlModel(self, stlFilePath):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stlFilePath)
        return self.drawPdSrc(reader)

    def removeActor(self, actor):
        self.renderer.RemoveActor(actor)

    def drawPoint(self, point, radius=2.0):
        src = vtk.vtkSphereSource()
        src.SetCenter(point.x, point.y, point.z)
        src.SetRadius(radius)
        return self.drawPdSrc(src)

    def drawNorVector(self, point, vector):
        src = vtk.vtkLineSource()
        src.SetPoint1(point.x, point.y, point.z)
        src.SetPoint2(point.x + vector.dx, point.y + vector.dy, point.z + vector.dz)
        return self.drawPdSrc(src)

    def drawSegment(self, seg):
        src = vtk.vtkLineSource()
        src.SetPoint1(seg.A.x, seg.A.y, seg.A.z)
        src.SetPoint2(seg.B.x, seg.B.y, seg.B.z)
        return self.drawPdSrc(src)

    def drawTriangle(self, tri):
        src = vtk.vtkLineSource()
        points = vtk.vtkPoints()
        points.InsertNextPoint((tri.A.x, tri.A.y, tri.A.z))
        points.InsertNextPoint((tri.B.x, tri.B.y, tri.B.z))
        points.InsertNextPoint((tri.C.x, tri.C.y, tri.C.z))
        points.InsertNextPoint((tri.A.x, tri.A.y, tri.A.z))
        src.SetPoints(points)
        return self.drawPdSrc(src)

    def drawPolyline(self, polyline):
        src = vtk.vtkLineSource()
        points = vtk.vtkPoints()
        for i in range(polyline.count()):
            pt = polyline.point(i)
            points.InsertNextPoint((pt.x, pt.y, pt.z))
        src.SetPoints(points)
        return self.drawPdSrc(src)

    pass


class StlModel:
    def __init__(self):
        self.triangles = []
        self.xMin = self.xMax = self.yMin = self.yMax = self.zMin = self.zMax = 0

    def getFaceNumber(self):
        return len(self.triangles)

    def getCoords(self, line):
        strs = line.lstrip().split('')
        cnt = len(strs)
        return float(strs[cnt - 3]), float(strs[cnt - 2]), float(strs[cnt - 1])

    def readStlFile(self, filepath):
        try:
            with open(filepath, 'r') as f:
                while True:
                    line = f.readline().strip('\n')
                    if line is None or line == '':
                        break
                    if 'facet normal' in line:
                        dx, dy, dz = self.getCoords(line)
                        N = Vector3D(dx, dy, dz)
                        f.readline()
                        A, B, C = Point3D(), Point3D(), Point3D()
                        A.x, A.y, A.z = self.getCoords(f.readline())
                        B.x, B.y, B.z = self.getCoords(f.readline())
                        C.x, C.y, C.z = self.getCoords(f.readline())
                        triangle = Triangle(A, B, C, N)
                        self.triangles.append(triangle)
        except Exception as err:
            print(err)

    def extraFromVtkStlReader(self, vtkStlReader):  # 提取面片信息
        vtkStlReader.Update()
        polydata = vtkStlReader.GetOutput()
        cells = polydata.GetPolys()
        cells.InitTraversal()
        while True:
            idList = vtk.vtkIdList()
            res = cells.GetNextCell(idList)
            if res == 0:
                break
            pnt3ds = []
            for i in range(idList.GetNumberOfIds()):
                id = idList.GetId(i)
                x, y, z = polydata.GetPoint(id)
                pnt3ds.append(Point3D(x, y, z))
            triangle = Triangle(pnt3ds[0], pnt3ds[1], pnt3ds[2])
            triangle.calcNormal()
            self.triangles.append(triangle)

    def getBounds(self):
        if len(self.triangles) == 0:
            return self.xMin, self.xMax, self.yMin, self.yMax, self.zMin, self.zMax
        else:
            self.xMin = self.xMax = self.triangles[0].A.x
            self.yMin = self.yMax = self.triangles[0].A.y
            self.yMin = self.yMax = self.triangles[0].A.z

        for t in self.triangles:
            self.xMin = min(t.A.x, t.B.x, t.C.x, self.xMin)
            self.yMin = min(t.A.y, t.B.y, t.C.y, self.yMin)
            self.zMin = min(t.A.z, t.B.z, t.C.z, self.zMin)
            self.xMax = max(t.A.x, t.B.x, t.C.x, self.xMax)
            self.yMax = max(t.A.y, t.B.y, t.C.y, self.yMax)
            self.zMax = max(t.A.z, t.B.z, t.C.z, self.zMax)

        return self.xMin, self.xMax, self.yMin, self.yMax, self.zMin, self.zMax


class SliceModel:
    """暂时用不到"""

    def __init__(self):
        pass

    def writeSlcFile(self, layers, path):
        """save slices in a  file"""
        pass

    def readSlcFile(self, path):
        """covert a slice file to slices"""
        pass

# renderer = vtk.vtkRenderer()
# renderer.SetBackground(0.95, 0.95, 0.95)
# window = vtk.vtkRenderWindow()
# window.AddRenderer(renderer)
# window.SetSize(900, 600)
# interactor = vtk.vtkRenderWindowInteractor()
# interactor.SetRenderWindow(window)
# interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
# interactor.Initialize()
#
# stlReader = vtk.vtkSTLReader()
# stlReader.SetFileName("done.stl")
#
# # source = vtk.vtkCubeSource()
# # mapper.SetInputConnection(source.GetOutputPort())
#
# outlineFilter = vtk.vtkOutlineFilter()
# outlineFilter.SetInputConnection(stlReader.GetOutputPort())
# outlineMapper = vtk.vtkPolyDataMapper()
# outlineMapper.SetInputConnection(outlineFilter.GetOutputPort())
# outlineActor = vtk.vtkActor()
# outlineActor.SetMapper(outlineMapper)
# outlineActor.GetProperty().SetColor(0.1, 0.1, 0.1)
# renderer.AddActor(outlineActor)
#
# stlMapper=vtk.vtkPolyDataMapper()
# stlMapper.SetInputConnection(stlReader.GetOutputPort())
# stlActor=vtk.vtkActor()
# stlActor.SetMapper(stlMapper)
# renderer.AddActor(stlActor)
#
# # trait = actor.GetProperty()
# # trait.SetRepresentationToWireframe()  # 设置线框模型
# # trait.SetRepresentationToPoints()  # 设置点模型
# # trait.SetRepresentationToSurface()  # 设置面模型
# # trait.SetColor(0.5, 0.6, 0.7)
# # trait.SetOpacity(0.5)
# # trait.EdgeVisibilityOn()
# # trait.SetEdgeColor(0, 0.8, 0)
# # trait.SetLineWidth(2)
#
#
# interactor.Start()
