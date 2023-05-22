from utils.math_utils import  World
from utils.wordfile_creator import save_worldfile,write_txt
from time import sleep
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"

def main (max_volume_density,scale,num):
    world = World(max_volume_density,scale=scale)

    while not world.full:
        try:
                obstacle = world.generate_random_obstacle()
                if world.valid_obstacle(obstacle.geom): world.add_obstacle(obstacle)
        except:
                sleep(1)
                pass
        

    name = f"HANNASSCAPES_random_obst_worldnum_{num}_scale_5_maxdens_" \
           f"{max_volume_density}_realdens_{world.volume_density()}".replace(".","dot")
    world.render(show=False,name=f"{name}.png",savepath="output/figures")
    save_worldfile(world,name=f"{name}.world",savepath="output/worlds")
    write_txt(name,"output/worldlist.txt")

if __name__ == "__main__":
    max_volume_density = 0.05
    for i in range(1,101):
        i+=80
        if (i-1) % 10 == 0 and i > 1:
            max_volume_density += 0.05

        main(max_volume_density=max_volume_density, scale=5, num=i)


