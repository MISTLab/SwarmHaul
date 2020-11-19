import math
import os
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
# plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg' # Linux only
ROBOT=[25,50,100]
OBJ_SIZE=[[2,2],[3.6, 6],[7.2, 12]]
offset_bots = 0.1

setting =1


# file_name = 'pos_300_2'

files = [f for f in os.listdir('.') if os.path.isfile(f) and f.endswith(".csv")]
# files = [f.split('.')[0] for f in files]
print(files)

def get_names(robot):
    name = ['time']
    for i in range(0,robot):
        name.append('id:'+str(i))
        name.append('x_'+str(i))
        name.append('y_'+str(i))
        name.append('z_'+str(i))
    name.append('obj_x')
    name.append('obj_y')
    name.append('obj_yaw')
    return name

def rotate(vector,theta):
    R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    return np.dot(R,vector)

def main(file_name):
    robots = ROBOT[setting]
    print(ROBOT[setting])
    obj_len = OBJ_SIZE[setting][0]-offset_bots
    obj_width=OBJ_SIZE[setting][1]-offset_bots
    names = get_names(robots)
    data = pd.read_csv(file_name , names=names, header=None, skiprows=1, engine='python')
    df = pd.DataFrame(data)

    # Only use some rows
    print("steps ", df.shape[0])
    # df = df[25000:25005]
    df = df.iloc[::100]

    df.drop('time', axis=1, inplace=True)
    # df.drop('obj_x', axis=1, inplace=True)
    # df.drop('obj_y', axis=1, inplace=True)
    # df.drop('obj_yaw', axis=1, inplace=True)
    df.drop(df.filter(regex='id').columns, axis=1, inplace=True)

    # Take every 3 cols (xyz) and combine into tuples (https://stackoverflow.com/questions/34052001/grouping-a-dataframe-and-applying-tuple)
    positionalDf = df.groupby(np.arange(len(df.columns)) // 3, axis=1).apply(lambda x: pd.Series([tuple(i) for i in x.values]))
    shape = positionalDf.shape
    # num_sheep = shape[1] - shape[1] % 50
    print("Running: ", file_name, " bots: ", shape[1]-1, end =" ")

    fig = plt.figure()  
    axis = plt.axes(xlim =(-15, 10),  
                    ylim =(-10, 10))
    # ax.set(xlim=(-3, 3), ylim=(-1, 1))
    plt.style.use("ggplot")

    positions_start = positionalDf[0:0+1].to_numpy().tolist()[0]
    sheep_pos_start = positions_start[:shape[1]-1]
    obj_pos = positions_start[shape[1]-1:]
    start_trans=axis.transData
    sheep_scat = axis.scatter(*( list(zip(*sheep_pos_start))[:-1] ), color='red')
    obj_lst=(( list(zip(*obj_pos))[:] ))
    
    patch = Rectangle((obj_pos[0][0]-(obj_len/2), obj_pos[0][1]-(obj_width/2)), obj_len, obj_width, fc='red', alpha=0.4)
    axis.add_patch(patch)
    # dog_scat = axis.scatter(*( list(zip(*sheep_pos_start))[:-1] ), color='green')

    def animate(t):
        positions = positionalDf[t:t+1].to_numpy().tolist()[0]
        pos_3s=positions
        positions = [x[0:2] for x in positions]
        sheep_pos_start = positions[:shape[1]-1]
        obj_pos = pos_3s[shape[1]-1:]
        obj_yaw=obj_pos[0][2]
        obj_pos = rotate([obj_pos[0][0], obj_pos[0][1]],-obj_yaw)
        patch.set_xy([obj_pos[0]-(obj_len/2), obj_pos[1]-(obj_width/2)])
        t2 = mpl.transforms.Affine2D().rotate_deg(np.rad2deg(obj_yaw)) + start_trans
        patch.set_transform(t2)
        
        # patch._angle = -np.rad2deg(obj_yaw)
        # dog_pos_start = positions[num_sheep:]
        sheep_scat.set_offsets(sheep_pos_start)
        # dog_scat.set_offsets(dog_pos_start)
        fig.suptitle(t, fontsize=12)

    # plt.draw()
    # plt.show()
    positions_over_time = animation.FuncAnimation(fig, animate, interval=5, frames=(shape[0] - 1))
    positions_over_time.save(file_name.split('.')[0] + '.mp4')
    print("frames: ", (shape[0] - 1))

    # Linux only 
    # writer = animation.FFMpegWriter(fps=30, codec='gif') 
    # positions_over_time.save('positions.mp4', writer = writer) fps = 30

for f in files:
    main(f)
