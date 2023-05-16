import shutil
import os

from utils.math_utils import ObjectTypes,euler_from_quaternion
import pyquaternion


def save_worldfile(world,name,savepath):
    if not os.path.exists(savepath):
        os.makedirs(savepath)
    worldpath = os.path.join(savepath,name)

    shutil.copyfile("beginningtemplate.world",worldpath)
    write_models(world,worldpath)
    write_closing(worldpath)

def get_gazebomodel_pose(object):
    roll,pitch,yaw = euler_from_quaternion(object.orientation) #inverse, since pyquaternion positive rotation is inverse to ROS
    x,y,z = object.center_point.x,object.center_point.y,object.center_point.z
    return f"{x} {y} {z} {roll} {pitch} {yaw}"

def get_gazebomodel_halfheight(object):
    return str(object.heightz/2)

def get_gazebomodel_boxsize(object):
    return f"{object.widthx} {object.widthy} {object.heightz}"

def write_model_sdf(object,worldpath,num):
    if object.obst_type == ObjectTypes.BOX:
        model_name = f"building_{num + 1}"
    else:
        TypeError("obect type not implemented yet")
    pose = get_gazebomodel_pose(object)
    halfheight = get_gazebomodel_halfheight(object)
    boxsize = get_gazebomodel_boxsize(object)

    with open("modeltemplate.world","r") as src:
        src_lines = src.readlines()

    with open (worldpath,"a") as dest:
        for line in src_lines:
            line = line.replace("BUILDING_NAME_REPLACE",model_name)
            line = line.replace("POSE_REPLACE",pose)
            line = line.replace("HALFHEIGHT_REPLACE",halfheight)
            line = line.replace("BOXSIZE_REPLACE",boxsize)
            dest.write(line)

def write_txt(name,txt):
    with open(txt,"a") as t:
        t.write(name+"\n")
def write_models(world,worldpath):
    for i,object in enumerate(world.objects):
        write_model_sdf(object,worldpath,i)


def write_closing(worldpath):
    with open(worldpath,"a") as file:
        file.write("   </world>\n")
        file.write("</sdf>")
