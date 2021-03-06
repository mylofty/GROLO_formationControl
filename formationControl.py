from robotClass import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from config import *
import math
import logging
import copy
import math
import os
import tensorflow as tf
from GridentDescentPy import PositionSolver
from GROLO_localization import setInitial_by_dvdistance, localization_gradient_descent, localization_GROLO


robot_Num = 0
beacon_Num = 0
communication_distance = 0
Beacon_1id = 0
Beacon_2id = 1
Beacon_3id = 10

AdjustTime = 0
times = 1
PictureName = "error_3/"

if not os.path.exists(PictureName):
    os.makedirs(PictureName)


def distance(coord1, coord2):
    return np.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def cmp_by_value(lhs):
    return lhs[1]

def show_old_new(oldpoints, targetpoints):
    global times
    fig = plt.figure(10+times)
    oldpoints = np.array(oldpoints)
    plt.title('blue:coord, red:position')
    plt.scatter(targetpoints[:, 0], targetpoints[:, 1], c='b')
    plt.scatter(oldpoints[:, 0], oldpoints[:, 1], c='r')
    for i in range(len(oldpoints)):
        plt.annotate(s=i, xy=(oldpoints[i, 0], oldpoints[i, 1]), xytext=(5, -5), textcoords='offset points')
        plt.annotate(s=i, xy=(targetpoints[i, 0], targetpoints[i, 1]), xytext=(-5, 5), textcoords='offset points')
    plt.plot(15, 15, 'black')
    plt.plot(0, 0, 'black')
    times = times + 1


def anim(list_points, points, name):
    fig = plt.figure()
    ax = plt.axes(xlim=(-2, 40), ylim=(-2, 40))
    length = len(list_points)
    new_points = np.array(list_points[0])
    ax.scatter(new_points[:, 0]+30/np.sqrt(2), new_points[:, 1]+30/np.sqrt(2), c='r')
    scat = ax.scatter(new_points[:, 0], new_points[:, 1], c='r')

    def update(frame_number):
        global new_points
        new_points = np.array(list_points[frame_number])
        new_points = new_points[:,0:2]
        # print('update ', frame_number, list_points[frame_number])
        scat.set_offsets(new_points)
        # for index in range(len(points)):
        #     scattext[index].set_position((new_points[index][0], new_points[index][1]))
        return scat,

    animate2 = animation.FuncAnimation(fig, update, frames=length, interval=20, blit=False)  # interval是每隔70毫秒更新一次，可以查看help
    animate2.save(name+"circle_shrink.gif", writer='imagemagick', fps=300)
    plt.show()


def point_curve(list_points, points, curvename):
    plt.figure(1060)
    # plt.title('collision_distance constraint 1.1 \ncommunication_distance constraint 4.2\ndistance between robot[0]\'s position and coordinate origin')
    # plt.xlabel('epoch')
    # plt.ylabel('distance')
    list_points = np.array(list_points)
    plt.scatter(points[:, 0], points[:, 1], c='r',label='initial position')
    plt.scatter(points[:, 0] + 30/np.sqrt(2), points[:, 1] + 30/np.sqrt(2), c='r',marker='+',label='object position')
    plt.scatter(list_points[len(list_points)-1, :, 0], list_points[len(list_points)-1, :, 1], c='b',label='real position')
    print('list_points',list_points)
    for index in range(11):
        plt.plot(list_points[:, index, 0], list_points[:, index, 1])
    plt.legend()
    plt.savefig(curvename + 'init_final.pdf')
    plt.show()








def set_initial_height(robots):
    center_x = (robots[Beacon_1id].coord[0] + robots[Beacon_3id].coord[0]) / 2
    center_y = (robots[Beacon_1id].coord[1] + robots[Beacon_3id].coord[1]) / 2
    for r in robots:
        r.initial_height = [r.coord[0]-center_x, r.coord[1] - center_y]

def initial_and_measured_neighbor(points, robots):
    global communication_distance
    '''
    use the current localization information modify the robots' neighbor information
    :param points: 
    :param robots: 
    :return: 
    '''
    center_x = (robots[Beacon_1id].coord[0] + robots[Beacon_3id].coord[0]) / 2
    center_y = (robots[Beacon_1id].coord[1] + robots[Beacon_3id].coord[1]) / 2

    for r in robots:
        r.isFinalPos = False
        r.nei_id = []
        r.measured_distance = {}
        r.myNeighbor = []
        r.nei_pos = []
        r.height = [r.coord[0]-center_x, r.coord[1] - center_y]

    for i in range(robot_Num):
        for j in range(i+1, robot_Num):
            # np.random.seed(12345)
            tempDistance = np.sqrt((points[i][0] - points[j][0]) ** 2
                                   + (points[i][1] - points[j][1]) ** 2)
            tempDistance = tempDistance + tempDistance * (np.random.random() * 0.06 - 0.03)  # 是否加噪声
            if tempDistance < communication_distance:
                robots[i].myNeighbor.append([j, tempDistance])
                robots[j].myNeighbor.append([i, tempDistance])
    for r in robots:
        r.myNeighbor = sorted(r.myNeighbor, key=cmp_by_value)
        for nei in r.myNeighbor:
            nid = nei[0]
            r.nei_id.append(nid)
            r.nei_pos.append(robots[nid].get_coord())
            r.measured_distance[nid] = nei[1]


def create_formation_topology():
    # np.random.seed(6)
    global beacon_Num
    global robot_Num
    global communication_distance
    global Beacon_1id, Beacon_2id, Beacon_3id
    beaconlist = np.loadtxt(os.path.join(folder, beacon_node_filename))
    points = np.loadtxt(os.path.join(folder, random_node_filename))
    robot_Num = points.shape[0]
    robots = [Robot(id=x) for x in range(robot_Num)]
    Beacon = np.array((beaconlist[0:len(beaconlist) - 1]), dtype=int)
    beacon_Num = len(Beacon)
    # we just consider 3 beacon now
    Beacon_1id = Beacon[0]
    Beacon_2id = Beacon[1]
    Beacon_3id = Beacon[2]
    communication_distance = beaconlist[-1]
    for index in Beacon:
        robots[index].set_beacon()
    for i in range(len(points)):
        robots[i].set_coord([points[i][0], points[i][1]])
        # robots[i].set_real_position([points[i][0], points[i][1]])

    # define triangle extension and fix it, so do not use triangle_extension_file.py
    robots[2].set_parents(0, 1)
    robots[3].set_parents(0, 2)
    robots[4].set_parents(1, 2)
    robots[5].set_parents(2, 3)
    robots[6].set_parents(4, 2)
    robots[7].set_parents(5, 3)
    robots[8].set_parents(4, 6)
    robots[9].set_parents(6, 8)
    robots[0].set_child(2)
    robots[0].set_child(3)
    robots[1].set_child(2)
    robots[1].set_child(4)
    robots[2].set_child(4)
    robots[2].set_child(5)
    robots[3].set_child(5)
    robots[3].set_child(7)
    robots[4].set_child(8)
    robots[4].set_child(6)
    robots[5].set_child(7)
    robots[5].set_child(10)
    robots[6].set_child(8)
    robots[6].set_child(9)
    robots[7].set_child(10)
    robots[8].set_child(9)
    robots[8].set_child(10)
    robots[9].set_child(10)
    robots[10].set_child(7)
    robots[10].set_child(8)
    robots[10].set_child(9)
    return points, robots


def adjust_height(points, robots, delta_H, velocity, direct):
    global AdjustTime
    adjust = False

    for r in robots:
        max_dx = velocity * (1+deltaV) * np.cos(direct + deltaD)
        min_dx = velocity * (1+deltaV) * np.cos(direct - deltaD)
        max_dy = velocity * (1+deltaV) * np.sin(direct + deltaD)
        min_dy = velocity * (1 + deltaV) * np.sin(direct - deltaD)
        dx_list = []
        dy_list = []
        for x in range(10):
            dx_list.append(min_dx + (max_dx - min_dx) * x / 10)
            dy_list.append(min_dy + (max_dy - min_dy) * x / 10)

        for d in range(len(dx_list)):
            # if any time the robot move will break the delta_H, it have to adjust dis
            [rd_x, rd_y] = np.array(r.initial_height) - (np.array(r.height)+np.array([dx_list[d], dy_list[d]]))
            [rrd_x, rrd_y] = np.array(r.initial_height) - (np.array(r.height))
            dis = (rd_x**2 + rd_y**2)**0.5
            if dis> delta_H:
                if adjust == False:
                    logging.error('u get adjust_height')
                    AdjustTime = AdjustTime + 1
                    adjust = True
                direct = math.atan2(rrd_y, rrd_x)
                print('robots[%d] adjust height, direct is %f, speed is %f' % (r.id, direct/math.pi * 180, dis))
                if r.move(dis, direct):
                    real_velocity = dis  # + np.random.random() * (dis * deltaV) * 2 - (dis * deltaV)
                    real_direct = direct  # + np.random.random() * deltaD * 2 - deltaD
                    dx = real_velocity * np.cos(real_direct)
                    dy = real_velocity * np.sin(real_direct)
                    points[r.id][0] = points[r.id][0] + dx
                    points[r.id][1] = points[r.id][1] + dy
                    initial_and_measured_neighbor(points, robots)
                break

def main():
    np.random.seed(12345)
    sess = tf.Session()
    psolver = PositionSolver(sess, 50, 0.0005)
    points, robots = create_formation_topology()
    set_initial_height(robots)
    initial_and_measured_neighbor(points, robots)

    # GROLO_LOCALIZATION
    setInitial_by_dvdistance(robots)
    localization_gradient_descent(robots, psolver, epochs=2)
    GROLO_position = localization_GROLO(robots, robot_Num - beacon_Num-1)
    print('grolo position', GROLO_position)

    # formation control
    epochs = 130
    global times, AdjustTime
    velocity = 0.3
    direct = np.pi / 4
    deltaH = 0.7
    list_points = []
    list_coords = []
    for epoch in range(epochs):
        times = times + 1
        print('this is %d picture' % times)
        oldcoords = []
        for r in robots:
            oldcoords.append(r.coord)
        list_coords.append(copy.deepcopy(oldcoords))
        list_points.append(copy.deepcopy(points))

        # adjust before moving beacon
        adjust_height(points, robots, deltaH, velocity, np.pi / 4)
        oldcoords = []
        for r in robots:
            oldcoords.append(r.coord)
        list_coords.append(copy.deepcopy(oldcoords))
        list_points.append(copy.deepcopy(points))

        # move beacon
        for r in robots:
            if r.isBeacon == True:
                if r.move(velocity, direct):
                    print("robot[{}] can moving".format(r.id))
                    real_velocity = velocity + np.random.random() * (velocity * deltaV) * 2 - (velocity * deltaV)
                    real_direct = direct + np.random.random() * deltaD * 2 - deltaD
                    dx = real_velocity * np.cos(real_direct)
                    dy = real_velocity * np.sin(real_direct)
                    points[r.id][0] = points[r.id][0] + dx
                    points[r.id][1] = points[r.id][1] + dy
                    initial_and_measured_neighbor(points, robots)
        # gradient_descent in beacon
        for r in robots:
            r.isBeacon = not r.isBeacon
        localization_gradient_descent(robots, psolver, epochs=20)
        for r in robots:
            r.isBeacon = not r.isBeacon
        oldcoords = []
        for r in robots:
            oldcoords.append(r.coord)
        list_coords.append(copy.deepcopy(oldcoords))
        list_points.append(copy.deepcopy(points))
        # print('oldcoord is ', oldcoords)
        # print('points is ', points)
        # show_old_new(oldcoords, points)


        # adjust before move non-beacon
        adjust_height(points, robots, deltaH, velocity, np.pi / 4)

        # save to list
        oldcoords = []
        for r in robots:
            oldcoords.append(r.coord)
        list_coords.append(copy.deepcopy(oldcoords))
        list_points.append(copy.deepcopy(points))
        # show_old_new(oldcoords, points)

        # move no beacon
        for r in robots:
            if r.isBeacon == False:
                if r.move(velocity, direct):
                    print('robot[{}] has move', r.id)
                    real_velocity = velocity + np.random.random() * (velocity * deltaV) * 2 - (velocity * deltaV)
                    real_direct = direct + np.random.random() * deltaD * 2 - deltaD
                    dx = real_velocity * np.cos(real_direct)
                    dy = real_velocity * np.sin(real_direct)
                    # print('robot[{}] dx, dy is [{}, {}]'.format(r.id, dx, dy), np.random.random(), np.random.random())
                    points[r.id][0] = points[r.id][0] + dx
                    points[r.id][1] = points[r.id][1] + dy
                    initial_and_measured_neighbor(points, robots)
        # GROLO_LOCALIZATION
        localization_gradient_descent(robots, psolver, epochs=10)
        GROLO_position = localization_GROLO(robots, robot_Num - beacon_Num - 1)
        # save to list
        oldcoords = []
        for r in robots:
            oldcoords.append(r.coord)
        list_coords.append(copy.deepcopy(oldcoords))
        list_points.append(copy.deepcopy(points))
        if epoch == epochs-1:
            show_old_new(oldcoords, points)
        # if distance(robots[Beacon_3id].coord,points[Beacon_3id,:]+30/np.sqrt(2)) < (deltaH):
        if robots[Beacon_3id].coord[0]>list_points[0][Beacon_3id, 0] + 30/np.sqrt(2) and \
                robots[Beacon_3id].coord[1]>list_points[0][Beacon_3id,1] + 30/np.sqrt(2):
            show_old_new(oldcoords, points)
            break

    # save the move information
    np.save(PictureName + " percent_list_points.npy", np.array(list_points))
    np.save(PictureName + " percent_list_coords.npy", np.array(list_coords))


    anim(list_coords, list_points[0], PictureName)
    point_curve(list_coords, list_points[0], PictureName + "list_coords")
    point_curve(list_points, list_points[0], PictureName + "list_points")
    plt.show()



def showadjust():
    adjustList = np.loadtxt('adjustList.npy')
    plt.xlabel('speed(m/s)')
    plt.ylabel('epoch')
    plt.plot(adjustList[0, :],adjustList[1, :], ls='-.', label='forward times')
    plt.plot(adjustList[0, :], adjustList[2, :], ls=':', label='adjust times')
    plt.plot(adjustList[0, :], adjustList[1, :]+adjustList[2, :], ls='--', label='whole times')
    plt.legend()
    plt.savefig('data/adjustCompare.pdf')
    plt.savefig('data/adjustCompare.eps')
    plt.show()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    main()
    # showadjust()





