from Geometry3D import *
from enum import Enum
import random
import numpy as np
import pyquaternion
import math

from utils.custom_geom3D import CustomRenderer,Obstacle


def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x,y,z,w=q.x,q.y,q.z,q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians
class ObjectTypes(Enum):
    TREE = 1
    BOX = 2
class World():
    def __init__(self,max_volume_density,scale):
        self.r = CustomRenderer()
        self.midpoint = [200, 0, 100]
        self.skybox = Skybox(shape=[600,600,200], midpoint=self.midpoint,r=self.r)
        self.drone_spawn_airspace = Cylinder(Point(0,0,0),5,200 * z_unit_vector(),n=15)
        self.goal_airspace = Cylinder(Point(400,0,0),5,200 * z_unit_vector(),n=15)
        self.objects = []
        self.max_volume_density = max_volume_density
        self.total_volume_obstacles = 0
        self.full = False
        self.fillup_tries = 0
        self.scale=scale
        self.spawn_phase = 1 # spawn phases for different object sizes. spawn big once first to boost performance
        self.skybox_histogramm = np.zeros(shape=(self.skybox.high_z-self.skybox.low_z+1,
                                                 self.skybox.high_y-self.skybox.low_y+1,
                                                 self.skybox.high_x-self.skybox.low_x+1)) # to block already tried points in space, for acceleration

    def generate_random_obstacle(self,blocks_only=True):
        obst_type = ObjectTypes.BOX if blocks_only else random.choice(list(ObjectTypes))
        if obst_type == ObjectTypes.BOX:
            pos = self.random_pos()
            if self.spawn_phase == 1:
                dim = np.clip(a=np.random.exponential(scale=100,size=(1,3)),a_min=4,a_max=150).astype(int).squeeze()
            elif self.spawn_phase == 2:
                dim = np.clip(a=np.random.exponential(scale=50,size=(1,3)),a_min=2,a_max=80).astype(int).squeeze()
            elif self.spawn_phase > 2:
                dim = np.clip(a=np.random.exponential(scale=3,size=(1,3)),a_min=1,a_max=10).astype(int).squeeze()
            random_orientation = pyquaternion.Quaternion.random()

            obst = Obstacle(pos, Vector(random_orientation.rotate((dim[0], 0, 0))),
                                  Vector(random_orientation.rotate((0, dim[1], 0))),
                                  Vector(random_orientation.rotate((0, 0, dim[2]))),
                          obst_type,random_orientation,dim)

        return obst

    def update_histogramm(self,pos):
        self.skybox_histogramm[self.world_2_histogramm_cos(round(pos.z),axis="z"),
        self.world_2_histogramm_cos(round(pos.y),axis="y"),
        self.world_2_histogramm_cos(round(pos.x),axis="x")] = 1
    def world_2_histogramm_cos(self,element,axis):
        if axis == "x":
            return element - self.skybox.low_x
        if axis == "y":
            return element - self.skybox.low_y
        if axis == "z":
            return element - self.skybox.low_z
    def histogramm_2_world_cos(self,element,axis):
        if axis == "x":
            return element + self.skybox.low_x
        if axis == "y":
            return element + self.skybox.low_y
        if axis == "z":
            return element + self.skybox.low_z

    def random_pos(self):
        distributed_z = int(np.clip(np.random.exponential(scale=self.skybox.high_z // self.scale, size=(1,)),
                                    a_min=self.skybox.low_z, a_max=self.skybox.high_z))

        height_extracted = self.skybox_histogramm[self.world_2_histogramm_cos(distributed_z,axis="z")]  # get slots from histrogram on z height
        open_slots = np.argwhere(height_extracted == 0) # get open slots
        # shuffle slots and choose x,y randomly from open slots
        np.random.shuffle(open_slots)
        random_x = self.histogramm_2_world_cos(open_slots[0,0],axis="x")
        random_y = self.histogramm_2_world_cos(open_slots[0,1],axis="y")
        pos = Point(random_x, random_y, distributed_z)
        return pos
    def volume_density(self):
        return self.total_volume_obstacles / self.skybox.volume
    def intersection_volume(self,inters):

        for obs in self.objects:
            if intersection(inters,obs.geom):
                inters = intersection(inters,obs.geom)
        return inters.volume()


    def valid_obstacle(self,candidate):
        if intersection(candidate.geom,self.drone_spawn_airspace): return False
        if intersection(candidate.geom,self.goal_airspace): return False
        for sky_bound in self.skybox.boundary_planes:
            if intersection(candidate.geom,sky_bound): return False
        sorted_objects = self.sort_objects_by_candidate_distance(candidate)
        for obs in sorted_objects:
            if intersection(candidate.geom,obs.geom):
                if intersection(candidate.geom.center_point,obs.geom): #block center point for further random spawns if intersects
                    self.update_histogramm(candidate.geom.center_point)
                return False #dont spawn candidate if intersects
        return True

    def sort_objects_by_candidate_distance(self,candidate):
        dis = [abs(np.linalg.norm(obj.center_point.asarray()-candidate.center_point.asarray())) for obj in self.objects]
        sorted_objects = np.stack((self.objects.copy(),dis),axis=1)
        sorted_objects = sorted_objects[sorted_objects[:,1].argsort()] # sort by distance, to boost intersection computation performance
        return list(sorted_objects[:,0])
    def add_obstacle(self,obstacle):
        self.objects.append(obstacle)
        self.total_volume_obstacles += obstacle.geom.volume()
        # check if obstacle would hurt max colume density condition
        print(self.volume_density())
        if self.volume_density() > self.max_volume_density:
            self.objects.pop(len(self.objects)-1)
            self.total_volume_obstacles -= obstacle.geom.volume()
            self.fillup_tries+=1
            if self.fillup_tries>5:
                self.full = True
            return
        if len(self.objects) >= 100:
            self.spawn_phase = 2
        if len(self.objects) >= 300:
            self.spawn_phase = 3
        self.update_histogramm(obstacle.center_point)
    def add_render(self):
        self.r.add((self.drone_spawn_airspace,'g',2))
        self.r.add((self.goal_airspace,'g',2))
        for object in self.objects:
            self.r.add((object.geom,'r',2))
    def render(self,show,name=None,savepath="figures"):
        self.skybox.add_render()
        self.add_render()
        if savepath:
            self.r.savefig(name,path=savepath)
        if show:
            self.r.show()
        self.r.clear_renderer()

class Skybox():
    def __init__(self,shape,midpoint,r):
        self.r = r
        low_z,high_z =midpoint[2]-shape[2]//2,midpoint[2]+shape[2]//2
        low_x,high_x =midpoint[0]-shape[0]//2,midpoint[0]+shape[0]//2
        low_y,high_y =midpoint[1]-shape[1]//2,midpoint[1]+shape[1]//2
        self.low_z,self.high_z = low_z,high_z
        self.low_x,self.high_x = low_x,high_x
        self.low_y,self.high_y = low_y,high_y
        self.boundary_planes = [Plane(Point(0,0,low_z),z_unit_vector()),
                                Plane(Point(0,0,high_z),z_unit_vector()),
                                Plane(Point(0,low_y,0),y_unit_vector()),
                                Plane(Point(0,high_y,0),y_unit_vector()),
                                Plane(Point(low_x,0,0),x_unit_vector()),
                                Plane(Point(high_x,0,0),x_unit_vector())]
        self.boundary_planes_RENDER = [ConvexPolygon((Point(low_x,low_y,low_z),
                                                      Point(high_x,low_y,low_z),
                                                      Point(low_x,high_y,low_z),
                                                      Point(high_x,high_y,low_z))),
                                       ConvexPolygon((Point(low_x, low_y, high_z),
                                                      Point(high_x, low_y, high_z),
                                                      Point(low_x, high_y, high_z),
                                                      Point(high_x, high_y, high_z))),
                                       ConvexPolygon((Point(low_x, low_y, low_z),
                                                      Point(high_x, low_y, low_z),
                                                      Point(low_x, low_y, high_z),
                                                      Point(high_x, low_y, high_z))),
                                       ConvexPolygon((Point(low_x, high_y, low_z),
                                                      Point(high_x, high_y, low_z),
                                                      Point(low_x, high_y, high_z),
                                                      Point(high_x, high_y, high_z)))
                                       ]

        self._volume = shape[0] * shape[1] * shape[2]


    @property
    def volume(self):
        return self._volume

    def add_render(self):
        for polygon in self.boundary_planes_RENDER:
            self.r.add((polygon,'b',1))
