from Geometry3D import *
from Geometry3D.render.renderer_matplotlib import MatplotlibRenderer
import os

def CustomRenderer(backend='matplotlib'):
    """
    **Input:**

    - backend: the backend of the renderer

    Only matplotlib is supported till now
    """
    if backend == 'matplotlib':
        return CustomMatplotlibRenderer()
    else:
        raise ValueError('Unknown backend %s' % (backend,))


class Obstacle():
    def __init__(self, basepoint, v1, v2, v3,obst_type,orientation,dimension):
        self.geom = Parallelepiped(basepoint,v1,v2,v3)
        self.obst_type = obst_type
        self._orientation = orientation
        self._dimension = dimension

    @property
    def center_point(self):
        return self.geom.center_point
    @property
    def orientation(self):
        return self._orientation

    @property
    def dimension(self):
        return self._dimension

    @property
    def widthx(self):
        return self._dimension[0]

    @property
    def widthy(self):
        return self._dimension[1]

    @property
    def heightz(self):
        return self._dimension[2]

    @orientation.setter
    def orientation(self, quaternion):
        self.orientation = quaternion

    @dimension.setter
    def dimension(self, dimension):
        self.dimension = dimension





class CustomMatplotlibRenderer(MatplotlibRenderer):
    def __init__(self):
        super(CustomMatplotlibRenderer, self).__init__()
        self.plt = None

    def create_plot(self):
        """
        Draw the image
        """
        from matplotlib import pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = Axes3D(fig)
        get_main_logger().info('Showing geometries with %d points, %d segments, %d arrows using matplotlib' % (
        len(self.point_set), len(self.segment_set), len(self.arrow_set)))
        for point_tuple in self.point_set:
            point = point_tuple[0]
            color = point_tuple[1]
            size = point_tuple[2]
            ax.scatter(point.x, point.y, point.z, c=color, s=size)

        for segment_tuple in self.segment_set:
            segment = segment_tuple[0]
            color = segment_tuple[1]
            size = segment_tuple[2]
            x = [segment.start_point.x, segment.end_point.x]
            y = [segment.start_point.y, segment.end_point.y]
            z = [segment.start_point.z, segment.end_point.z]
            ax.plot(x, y, z, color=color, linewidth=size)

        for arrow_tuple in self.arrow_set:
            x, y, z, u, v, w, length = arrow_tuple[0].get_tuple()
            color = arrow_tuple[1]
            size = arrow_tuple[1]
            ax.quiver(x, y, z, u, v, w, color=color, length=length)

        self.plt = plt
    def show(self):
        if not self.plt: self.create_plot()
        self.plt.show()
    def savefig(self,name,path="figures"):
        if not self.plt: self.create_plot()
        if not os.path.isdir(path):
            os.makedirs(path)
        self.plt.draw()
        self.plt.savefig(os.path.join(path,name),dpi=1000)
    def clear_renderer(self):
        self.plt.figure().clear()
        self.plt.close()
        self.plt.cla()
        self.plt.clf()
        self.plt = None

