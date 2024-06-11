import math

epsilon = 1e-7
epsilonSquare = epsilon * epsilon


class Point3D:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w

    def __str__(self):
        return 'Point3D: %s, %s, %s' % (self.x, self.y, self.z)

    def clone(self):  # clone to a new point
        return Point3D(self.x, self.y, self.z, self.w)

    def pointTo(self, other):  # return a vector from this point to the other
        return Vector3D(other.x - self.x, other.y - self.y, other.z - self.z)

    def translate(self, vec):  # print the translated point
        self.x, self.y, self.z = self.x + vec.x, self.y + vec.y, self.z + vec.z

    def translated(self, vec):  # return a translated point
        return Point3D(self.x + vec.dx, self.y + vec.dy, self.z + vec.dz)

    def multiplied(self, m):  # right multiply a matrix
        x = self.x * m.a[0][0] + self.y * m.a[1][0] + self.z * m.a[2][0] + self.w * m.a[3][0]
        y = self.x * m.a[0][1] + self.y * m.a[1][1] + self.z * m.a[2][1] + self.w * m.a[3][1]
        z = self.x * m.a[0][2] + self.y * m.a[1][2] + self.z * m.a[2][2] + self.w * m.a[3][2]
        return Point3D(x, y, z)

    def distance(self, other):
        return self.pointTo(other).length()

    def distanceSquare(self, other):
        return self.pointTo(other).lengthSquare()

    def middle(self, other):  # return the middle point from the targets
        return Point3D((self.x + other.x) / 2, (self.x + other.y) / 2, (self.x + other.y) / 2)

    def isCoincide(self, other, dis2=epsilonSquare):
        return True if self.pointTo(other).lengthSquare() < dis2 else False

    def isIdentical(self, other):
        return True if self.x == other.x and self.y == other.y and self.z == other.z else False

    def __add__(self, other):
        return self.translated(other)

    def __sub__(self, other):
        return other.pointTo(self) if isinstance(other, Point3D) else self.translated(other.reversed())

    def __mul__(self, other):
        return self.multiplied(other)


class Vector3D:
    def __init__(self, dx=0.0, dy=0.0, dz=0.0, dw=0.0):
        self.dx, self.dy, self.dz, self.dw = dx, dy, dz, dw

    def __str__(self):
        return 'Vector3D: %s, %s, %s' % (self.dx, self.dy, self.dz)

    def clone(self):
        return Vector3D(self.dx, self.dy, self.dz, self.dw)

    def reverse(self):
        self.dx, self.dy, self.dz = -self.dx, -self.dy, -self.dz

    def reversed(self):
        return Vector3D(-self.dx, -self.dy, -self.dz)  # !!!dw=0?

    def dotProduct(self, vec):
        return self.dx * vec.dx + self.dy * vec.dy + self.dz * vec.dz

    def crossProduct(self, vec):
        dx = self.dy * vec.dz - self.dz * vec.dy
        dy = self.dz * vec.dx - self.dx * vec.dz
        dz = self.dx * vec.dy - self.dy * vec.dx
        return Vector3D(dx, dy, dz)

    def amplify(self, f):
        self.dx, self.dy, self.dz = self.dx * f, self.dy * f, self.dz * f

    def amplified(self, f):
        return Vector3D(self.dx * f, self.dy * f, self.dz * f)

    def lengthSquare(self):
        return self.dx * self.dx + self.dy * self.dy + self.dz * self.dz
        # return self.dotProduct(self)

    def length(self):
        return math.sqrt(self.lengthSquare())

    def normalize(self):
        len = self.length()
        self.dx, self.dy, self.dz = self.dx / len, self.dy / len, self.dz / len

    def normalized(self):
        len = self.length()
        return Vector3D(self.dx / len, self.dy / len, self.dz / len)

    def isZeroVector(self):
        return self.length() == 0.0  # epsilon?

    def multiplied(self, m):
        dx = self.dx * m.a[0][0] + self.dy * m.a[1][0] + self.dz * m.a[2][0] + self.dw * m.a[3][0]
        dy = self.dx * m.a[0][1] + self.dy * m.a[1][1] + self.dz * m.a[2][1] + self.dw * m.a[3][1]
        dz = self.dx * m.a[0][2] + self.dy * m.a[1][2] + self.dz * m.a[2][2] + self.dw * m.a[3][2]
        return Vector3D(dx, dy, dz)

    def isParallel(self, other):
        return self.crossProduct(other).isZeroVector()

    def getAngle(self, other):
        v1, v2 = self.normalized(), other.normalized()
        dotPro = v1.dotProduct(v2)
        if dotPro > 1:
            dotPro = 1
        elif dotPro < 1:
            dotPro = -1
        return math.acos(dotPro)

    def getOrthoVector2D(self):
        if self.dx == 0:
            return Vector3D(1, 0, 0).normalized()
        else:
            return Vector3D(-self.dy / self.dx, 1, 0).normalized()

    def getAngle2D(self):  # calculate the angle between the vector and x-axis in XY-plane
        rad = self.getAngle(Vector3D(1, 0, 0))
        if self.dy < 0:
            rad = math.pi * 2.0 - rad
        return rad

    def __add__(self, other):
        return Vector3D(self.dx + other.dx, self.dy + other.dy, self.dz + other.dz)

    def __sub__(self, other):
        return self + other.reversed()

    def __mul__(self, other):
        return self.multiplied(other)


class Triangle:
    def __init__(self, A, B, C, N=Vector3D(0, 0, 0)):
        self.A, self.B, self.C, self.N = A.clone(), B.clone(), C.clone(), N.clone()
        self.zs = []
        self.pntMin = None
        self.pntMax = None

    def __str__(self):
        s1 = 'Triangle\nA:%s,%s,%s\nB:%s,%s,%s\nC:%s,%s,%s\n' % (
            self.A.x, self.A.y, self.A.z, self.B.x, self.B.y, self.B.z, self.C.x, self.C.y, self.C.z)
        s2 = "N:%s,%s,%s" % (self.N.dx, self.N.dy, self.N.dz)
        return s1 + s2

    def zMinPnt(self):
        if self.pntMin is None:
            zMin = self.A.z
            pntMin = self.A
            if self.B.z < zMin:
                zMin = self.B.z
                pntMin = self.B
            if self.C.z < zMin:
                zMin = self.C.z
                pntMin = self.C
            self.pntMin = pntMin
        return self.pntMin

    def zMaxPnt(self):
        if self.pntMax is None:
            zMax = self.A.z
            pntMax = self.A
            if self.B.z > zMax:
                zMax = self.B.z
                pntMax = self.B
            if self.C.z > zMax:
                zMax = self.C.z
                pntMax = self.C
            self.pntMax = pntMax
        return self.pntMax

    def calcNormal(self):
        v1 = self.A.pointTo(self.B)
        v2 = self.A.pointTo(self.C)
        n = v1.crossProduct(v2)
        self.N = n.normalized()
        return self.N

    def draw(self, va):
        va.drawSegment(Segment(self.A, self.B))
        va.drawSegment(Segment(self.A, self.C))
        va.drawSegment(Segment(self.B, self.C))


class Layer:
    def __init__(self, z):
        self.z = z
        self.segments = []
        self.contours = []  # 封闭截交线


class Matrix3D:
    def __init__(self):
        self.a = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    def __str__(self):
        return 'Matrix3D:\n%s\n%s\n%s\n%s' % (self.a[0], self.a[1], self.a[2], self.a[3])

    def makeIdentical(self):  # matrix unitization
        self.a = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    def multiplied(self, other):
        m = Matrix3D()
        for i in range(4):
            for j in range(4):
                m.a[i][j] = (self.a[i][0] * other.a[0][j] + self.a[i][1] * other.a[1][j] +
                             self.a[i][2] * other.a[2][j] + self.a[i][3] * other.a[3][j])
        return m

    def getDeterminant(self):
        pass

    def getReverseMatrix(self):
        pass

    @staticmethod
    def createTranslateMatrix(dx, dy, dz):
        m = Matrix3D()
        m.a[3][0], m.a[3][1], m.a[3][2] = dx, dy, dz
        return m

    @staticmethod
    def createScalMatrix(sx, sy, sz):
        m = Matrix3D()
        m.a[0][0], m.a[1][1], m.a[2][2] = sx, sy, sz
        return m

    @staticmethod
    def createRotateMatrix(axis, angle):
        m = Matrix3D()
        sin, cos = math.sin(angle), math.cos(angle)
        if axis == "X" or axis == 'x':
            m.a[1][1], m.a[1][2], m.a[2][1], m.a[2][2] = cos, sin, -sin, cos
        elif axis == "Y" or axis == 'y':
            m.a[0][0], m.a[0][2], m.a[2][0], m.a[2][2] = cos, -sin, sin, cos
        elif axis == 'Z' or axis == 'z':
            m.a[0][0], m.a[0][1], m.a[1][0], m.a[1][1] = cos, sin, -sin, cos
        return m

    @staticmethod
    def createMirrorMatrix(point, normal):
        pass

    def __mul__(self, other):
        return self.multiplied(other)

    def __add__(self, other):
        pass

    def __sub__(self, other):
        pass


class Line:
    def __init__(self, P, V):
        self.P = P.clone()
        self.V = V.clone().normalized()

    def __str__(self):
        return "Line\nP %s\nV %s\n" % (str(self.P), str(self.V))


class Ray:
    def __init__(self, P, V):
        self.P = P.clone()
        self.V = V.clone().normalized()

    def __str__(self):
        return "Ray\nP %s\nV %s\n" % (str(self.P), str(self.V))


class Segment:
    def __init__(self, A, B):
        self.A, self.B = A.clone(), B.clone()

    def __str__(self):
        return "Segment\nA %s\nB %s\n" % (str(self.A), str(self.B))

    def length(self):
        return self.A.distance(self.B)

    def direction(self):
        return self.A.pointTo(self.B)

    def swap(self):
        self.A, self.B = self.B, self.A


class PolyLine:
    def __init__(self):
        self.points = []

    def __str__(self):
        if self.count() > 0:
            return 'Polyline\n Point number:%s\nStart %s\nEnd %s\n' % (self.count(),
                                                                       str(self.startPoint()), str(self.endPoint()))
        else:
            return "Polyline \n Point number: 0\n"

    def clone(self):
        poly = PolyLine()
        for item in self.points:
            poly.addPoint(item.clone())
        return poly

    def count(self):
        return len(self.points)

    def addPoint(self, pt):
        self.points.append(pt)

    def addTuple(self, item):
        self.points.append(Point3D(item[0], item[1], item[2]))

    def raddPoint(self, pt, index=0):
        self.points.insert(index, pt)

    def appendSegment(self, seg):
        if self.count() == 0:
            self.points.append(seg.A)
            self.points.append(seg.B)
        else:
            if seg.A.isCoincide(self.endPoint()):
                self.addPoint(seg.B)
            elif seg.B.isCoincide(self.endPoint()):
                self.addPoint(seg.A)
            elif seg.A.isCoincide(self.startPoint()):
                self.raddPoint(seg.B)
            elif seg.B.isCoincide(self.startPoint()):
                self.raddPoint(seg.A)
            else:
                # print error?
                return False
        return True

    def removePoint(self, index):
        return self.points.pop(index)

    def point(self, index):
        return self.points[index]

    def startPoint(self):
        return self.points[0]

    def endPoint(self):
        return self.points[-1]

    def isClosed(self):
        if self.count() <= 2:
            return False
        return self.startPoint().isCoincide(self.endPoint())

    def reverse(self):
        sz = self.count()
        for i in range(int(sz / 2)):
            self.points[i], self.points[sz - 1 - i] = self.points[sz - 1 - i], self.points[i]

    def getArea(self):
        # assert self.isClosed()
        area = 0.0
        for i in range(self.count() - 1):
            area += 0.5 * (self.points[i].x * self.points[i + 1].y - self.points[i + 1].x * self.points[i].y)
        return area

    def makeCCW(self):
        if self.getArea() < 0:
            self.reverse()

    def makeCW(self):
        if self.getArea() > 0:
            self.reverse()

    def isCCW(self):
        return True if self.getArea() > 0 else False

    def translate(self, vec):
        for i in range(len(self.points)):
            self.points[i].translate(vec)

    @staticmethod
    def writePolyline(path, polyline):
        try:
            with open(path, 'w') as f:
                f.write('%s\n' % polyline.count())
                for pt in polyline.points:
                    txt = '%s,%s,%s\n' % (pt.x, pt.y, pt.z)
                    f.write(txt)
        except Exception as err:
            print(err)
        # f = None
        # try:
        #     f=open(path,'w')
        #     f.write('%s\n'%polyline.count())
        #     for pt in polyline.points:
        #         txt = '%s,%s,%s\n'%(pt.x, pt.y, pt.z)
        #         f.write(txt)
        # except Exception as ex:
        #     print(ex)
        # finally:
        #     if f:
        #         f.close()

    @staticmethod
    def readPolyline(path):
        try:
            with open(path, 'r') as f:
                poly = PolyLine()
                number = int(f.readline())
                for i in range(number):
                    txt = f.readline()
                    txts = txt.split(',')
                    x, y, z = float(txts[0]), float(txts[1]), float(txts[2])
                    poly.addPoint(Point3D(x, y, z))
                return poly
        except Exception as err:
            print(err)


class Plane:
    def __init__(self, P, N):
        self.P = P.clone()  # the point inside the plane
        self.N = N.clone().normalized()  # normal vector

    def __str__(self):
        return "Plane \n%s\n%s\n" % (str(self.P), str(self.N))

    def toFormula(self):  # Ax+By+Cz+D=0  from  N(X-P)=0
        A, B, C = self.N.dx, self.N.dy, self.N.dz
        D = -self.N.dx * self.P.x - self.N.dy * self.P.dy - self.N.dz * self.P.dz
        return A, B, C, D

    def intersect(self, other):
        dir = self.N.crossProduct(other.N)
        if dir.isZeroVector():
            return None
        else:
            x, y, z = 0, 0, 0
            A1, B1, C1, D1 = self.toFormula()
            A2, B2, C2, D2 = other.toFormula()
            if B2 * C1 - B1 * C2 != 0:
                y = -(-C2 * D1 + C1 * D2) / (B2 * C1 - B1 * C2)
                z = -(B2 * D1 - B1 * D2) / (B2 * C1 - B1 * C2)
            elif A2 * C1 - A1 * C2 != 0:
                x = -(-C2 * D1 + C1 * D2) / (A2 * C1 - A1 * C2)
                z = -(A2 * D1 - A1 * D2) / (A2 * C1 - A1 * C2)
            elif A2 * B1 - A1 * B2 != 0:
                x = -(-B2 * D1 + B1 * D2) / (A2 * B1 - A1 * B2)
                y = -(A2 * D1 - A1 * D2) / (A2 * B1 - A1 * B2)
            else:
                return None
        return Line(Point3D(x, y, z), dir.normalized())

    @staticmethod
    def zPlane(z):
        return Plane(Point3D(0, 0, z), Vector3D(0, 0, 1.0))


class SweepPlane:
    def __init__(self):
        self.triangles = []

    pass


# class IntersectStl_sweep:
# def __init__(self, stlModel, layerThk, zBase):
#     self.stlModel = stlModel
#     self.layerThk = layerThk
#     self.layers = []
#     self.intersect()
#     self.z_base = zBase
#
# def intersect(self):
#     triangles = self.stlModel.triangles
#     triangles.sort(key=lambda t: t.zMinPnt().z)
#     zs = self.genLayerHeights()
#     k = 0
#     sweep = SweepPlane()
#     for z in zs:
#         for i in range(len(sweep.triangles) - 1, -1, -1):
#             if z > sweep.triangles[i].zMaxPnt().z:
#                 del sweep.triangles[i]
#         for i in range(k, len(triangles)):
#             tri = triangles[i]
#             if tri.zMinPnt().z <= z <= tri.zMaxPnt().z:
#                 sweep.triangles.append(tri)
#             elif tri.zMinPnt().z > z:
#                 k = i
#                 break
#         layer = Layer(z)
#         for triangle in sweep.triangles:
#             seg = intersectTriangleZPlane(triangle.z)
#             if seg is not None:
#                 layer.segments.append(seg)
#         self.layers.append(layer)
#
# def genLayerHeights(self):
#     xMin, xMax, yMin, yMax, zMin, zMax = self.stlModel.getBounds()
#     print("Bounds: X [{},{}], Y [{},{}], Z[{},{}]".format(xMin, xMax, yMin, yMax, zMin, zMax))
#     if self.z_base is None:
#         zs, z = [], zMin + self.layerThk
#     else:
#         zs, z = [], self.z_base
#     while z < zMax:
#         zs.append(z)
#         z += self.layerThk
#     return zs

def nearZero(x):
    return True if math.fabs(x) < epsilon else False


def distance(obj1, obj2):  # point before line and line before plane
    if isinstance(obj1, Point3D) and isinstance(obj2, Line):
        P, Q, V = obj2.P, obj1, obj2.V
        t = P.pointTo(Q).dotProduct(V)
        R = P + V.amplified(t)
        return Q.distance(R)
    elif isinstance(obj1, Point3D) and isinstance(obj2, Ray):
        P, Q, V = obj2.P, obj1, obj2.V
        t = P.pointTo(Q).dotProduct(V)
        if t >= 0:
            R = P + V.amplified(t)
            return Q.distance(R)
        return Q.distance(P)
    elif isinstance(obj1, Point3D) and isinstance(obj2, Segment):
        Q, P, P1, V = obj1, obj2.A, obj2.B, obj2.direction().normalized()
        L = obj2.length()
        t = P.pointTo(Q).dotProduct(V)
        if t <= 0:
            return Q.distance(P)
        elif t >= L:
            return Q.distance(P1)
        else:
            R = P + V.amplified(t)
            return Q.distance(R)
    elif isinstance(obj1, Point3D) and isinstance(obj2, Plane):
        P, Q, N = obj2.P, obj1, obj2.N
        angle = N.getAngle(P.pointTo(Q))
        return P.distance(Q) * math.cos(angle)
    elif isinstance(obj1, Line) and isinstance(obj2, Line):
        P1, V1, P2, V2 = obj1.P, obj1.V, obj2.P, obj2.V
        N = V1.crossProduct(V2)
        if N.isZeroVector():
            return distance(P1, obj2)
        return distance(P1, Plane(P2, N))
    elif isinstance(obj1, Line) and isinstance(obj2, Plane):
        return distance(obj1.P, obj2) if obj1.V.dotProduct(obj2.N) == 0 else 0
    elif isinstance(obj1, Ray) and isinstance(obj2, Plane):
        pass
    elif isinstance(obj1, Segment) and isinstance(obj2, Plane):
        pass
    pass


def intersectLineLine(line1, line2):
    # assert (isinstance(line1, Line) and isinstance(line2, Line)), 'Please ensure the input types are both Line'
    P1, V1, P2, V2 = line1.P, line1.V, line2.P, line2.V
    P1P2 = P1.pointTo(P2)
    deno = V1.dy * V2.dx - V1.dx * V2.dy
    if deno != 0:
        t1 = -(-P1P2.dy * V2.dx + P1P2.dx * V2.dy) / deno
        t2 = -(-P1P2.dy * V1.dx + P1P2.dx * V1.dy) / deno
        return P1 + V1.amplified(t1), t1, t2
    else:
        deno = V1.dz * V2.dy - V1.dy * V2.dz
        if deno != 0:
            t1 = -(-P1P2.dz * V2.dy + P1P2.dy * V2.dz) / deno
            t2 = -(-P1P2.dz * V1.dy + P1P2.dy * V1.dz) / deno
            return P1 + V1.amplified(t1), t1, t2
    return None, 0, 0


def intersectSegmentPlane(seg, plane):
    A, B, P, N = seg.A, seg.B, plane.P, plane.N
    V = A.pointTo(B)
    PA = P.pointTo(A)
    if V.dotProduct(N) == 0:
        return None  # 线段在面内 交点很多 取无穷点 也即无交点
    else:
        t = -(PA.dotProduct(N)) / V.dotProduct(N)
        if 0 <= t <= 1:
            return A + (V.amplified(t))
    return None


def intersectTrianglePlane(triangle, plane):
    AB = Segment(triangle.A, triangle.B)
    AC = Segment(triangle.A, triangle.C)
    BC = Segment(triangle.B, triangle.C)
    c1 = intersectSegmentPlane(AB, plane)
    c2 = intersectSegmentPlane(AC, plane)
    c3 = intersectSegmentPlane(BC, plane)

    if c1 is None:
        if c2 is not None and c3 is not None:
            if c2.distance(c3) != 0.0:
                return Segment(c2, c3)
    elif c2 is None:
        if c1 is not None and c3 is not None:
            if c1.distance(c3) != 0.0:
                return Segment(c1, c3)
    elif c3 is None:
        if c1 is not None and c2 is not None:
            if c1.distance(c2) != 0.0:
                return Segment(c1, c2)

    elif c1 is not None and c2 is not None and c3 is not None:
        return Segment(c1, c3) if c1.isIdentical(c2) else Segment(c1, c2)

    return None


def intersectTriangleZPlane(triangle, z):
    if triangle.zMinPnt().z > z:
        return None
    if triangle.zMaxPnt().z < z:
        return None
    return intersectTrianglePlane(triangle, Plane.zPlane(z))


def intersect(obj1, obj2):
    if isinstance(obj1, Line) and isinstance(obj2, Line):
        P, t1, t2 = intersectLineLine(obj1, obj2)
        return P
    elif isinstance(obj1, Segment) and isinstance(obj2, Segment):
        line1, line2 = Line(obj1.A, obj1.direction()), Line(obj2.A, obj2.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None:
            if 0 <= t1 <= obj1.length() and 0 <= t2 <= obj2.length():
                return P
        return None
    elif isinstance(obj1, Line) and isinstance(obj2, Segment):
        line1, line2 = obj1, Line(obj2.A, obj2.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None and 0 <= t2 <= obj2.length():
            return P
        return None
    elif isinstance(obj1, Line) and isinstance(obj2, Ray):
        pass
    elif isinstance(obj1, Ray) and isinstance(obj2, Segment):
        pass
    elif isinstance(obj1, Ray) and isinstance(obj2, Ray):
        pass
    elif isinstance(obj1, Line) and isinstance(obj2, Plane):
        P0, V, P1, N = obj1.P, obj1.V, obj2.P, obj2.N
        dotPro = V.dotProduct(N)
        if dotPro != 0:
            t = P0.pointTo(P1).dotProduct(N) / dotPro
            return P0 + V.amplified(t)
        return None
    elif isinstance(obj1, Line) and isinstance(obj2, Triangle):
        tem_plane = Plane(obj2.A, obj2.calcNormal())
        P = intersect(obj1, tem_plane)
        if P:  # P is demonstrated in plane ABC
            PA = Vector3D(dx=obj2.A.x - P.x, dy=obj2.A.y - P.y, dz=obj2.A.z - P.z)
            PB = Vector3D(dx=obj2.B.x - P.x, dy=obj2.B.y - P.y, dz=obj2.B.z - P.z)
            PC = Vector3D(dx=obj2.C.x - P.x, dy=obj2.C.y - P.y, dz=obj2.C.z - P.z)
            t1 = PA.crossProduct(PB)
            t2 = PB.crossProduct(PC)
            t3 = PC.crossProduct(PA)
            if t1.dotProduct(t2) >= 0 and t1.dotProduct(t3) >= 0 and t2.dotProduct(t3) >= 0:
                return P, obj2.calcNormal()
        return None
    elif isinstance(obj1, Ray) and isinstance(obj2, Plane):
        pass
    elif isinstance(obj1, Segment) and isinstance(obj2, Plane):
        return intersectSegmentPlane(obj1, obj2)
    pass


def pointOnRay(p, ray):
    v = ray.P.pointTo(p)
    return True if v.dotProduct(ray.V) >= 0 and v.crossProduct(ray.V).isZero.Vector() else False


def pointInPolygon(p, polygon):  # return 1: inside | 0: outside | -1 on the side
    passCount = 0
    ray = Ray(p, Vector3D(1, 0, 0))
    segments = []
    for i in range(polygon.count() - 1):
        seg = Segment(polygon.point(i), polygon.point(i + 1))
        segments.append(seg)
    for seg in segments:
        line1, line2 = Line(ray.P, ray.V), Line(seg.A, seg.direction())
        P, t1, t2 = intersectLineLine(line1, line2)
        if P is not None:
            if nearZero(t1):
                return -1
            elif seg.A.y != p.y and seg.B.y != p.y and t1 > 0 and 0 < t2 < seg.length():
                passCount += 1
                upSegments, downSegments = [], []
                for seg in segments:
                    if seg.A.isIdentical(ray.P) or seg.B.isIdentical(ray.P):
                        return -1
                    elif pointOnRay(seg.A, ray) ^ pointOnRay(seg.B, ray):
                        if seg.A.y >= p.y and seg.B.y >= p.y:
                            upSegments.append(seg)
                        elif seg.A.y <= p.y and seg.B.y <= p.y:
                            downSegments.append(seg)
                passCount += min(len(upSegments), len(downSegments))
                return 1 if passCount % 2 == 1 else 0


def intersectStl_brutal(stlModel, layerThk, zBase=None):
    """Brutal force method"""
    layers = []
    xMin, xMax, yMin, yMax, zMin, zMax = stlModel.getBounds()
    print("Bounds: X [{},{}], Y [{},{}], Z[{},{}]".format(xMin, xMax, yMin, yMax, zMin, zMax))
    if zBase is None:
        z = zMin + layerThk
    else:
        z = float(zBase)
    while z < zMax:
        layer = Layer(z)
        for tri in stlModel.triangles:
            seg = intersectTriangleZPlane(tri, z)
            if seg is not None:
                layer.segments.append(seg)
        layers.append(layer)
        z += layerThk
    return layers


def intersectStl_sweep(stlModel, layerThk, zBase=None):
    pass
    # return IntersectStl_sweep(stlModel, layerThk, zBase).layers


def linkSeg_brutal(segs):
    segs = list(segs)
    contours = []
    while len(segs) > 0:
        contour = PolyLine()
        contours.append(contour)
        while len(segs) > 0:  # 若无法闭合或连接 会陷入死循环2023.9.15
            print(len(segs))
            for seg in segs:
                if contour.appendSegment(seg):
                    segs.remove(seg)
                    break
            if contour.isClosed():
                break
    return contours


def adjustPolygonDirs(polygons):
    for i in range(len(polygons)):
        pt = polygons[i].startPoint()
        insideCount = 0
        for j in range(len(polygons)):
            if j == i:
                continue
            restPoly = polygons[j]
            if 1 == pointInPolygon(pt, restPoly):
                insideCount += 1
        if insideCount % 2 == 0:
            polygons[i].makeCCW()
        else:
            polygons[i].makeCW()


def reverse_GcodeLines(filepath="test.txt", savepath="result.txt"):
    content = []
    with open(filepath) as f:
        for row in f.readlines():
            content.append(row)
    content.reverse()
    with open(savepath, "w") as f:
        for row in content:
            f.write(row)


def interval_two_points(x1, x2, y1, y2, z1=0.0, z2=0.0, res=10, z_output=False):
    inter_point = []
    tem_point = [0, 0, 0]
    for i in range(int(res)):
        tem_point[0] = (x2 - x1) / res * i + x1
        tem_point[1] = (y2 - y1) / res * i + y1
        tem_point[2] = (z2 - z1) / res * i + z1
        inter_point.append(tem_point.copy())
    for i in range(int(res)):
        if z_output:
            print("G1 X{} Y{} Z{} F300".format(inter_point[i][0], inter_point[i][1], inter_point[i][2]))
        else:
            print("G1 X{} Y{} F300".format(inter_point[i][0], inter_point[i][1]))


def read_2D_Gcode(Gcode_filepath='gcode.txt', offset_x=0, offset_y=0, scale=1):
    points = []
    try:
        with open(Gcode_filepath) as f:
            rows = f.readlines()
            ini_x = float(rows[0].split(' ')[1][1:])
            ini_y = float(rows[0].split(' ')[2][1:])
            for row in rows:
                points.append(Point3D(x=(float(row.split(' ')[1][1:]) - offset_x)*scale,
                                      y=(float(row.split(' ')[2][1:]) - offset_y)*scale))
    except Exception as e:
        print(e)
    return points


def projection_point2stlModel(points, stlModel, dz=1.0):
    inter_point = []
    normal_vector = []
    for point in points:
        line = Line(point, Vector3D(dz=dz))
        for tri in stlModel.triangles:  # 可优化 缩小求交范围
            if intersect(line, tri) is not None:
                interpoint, normalvector = intersect(line, tri)
                inter_point.append(interpoint)
                normal_vector.append(normalvector)
                break  # 需检查 1027是为了防止模型bug多交点 事实上只应由一个交点
    if len(inter_point):
        return inter_point,normal_vector
    else:
        return None
