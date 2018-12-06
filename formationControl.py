from robotClass import *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from config import *
import math
import logging
import copy
import math
import tensorflow as tf
from GridentDescentPy import PositionSolver
from GROLO_localization import setInitial_by_dvdistance, localization_gradient_descent, localization_GROLO

AdjustTime = 0
times = 1


Beacon_1List = []

Robot_list = []
Robot_listp1 = []
Robot_listp2 = []

Robot2_listp1 = []
Robot2_listp2 = []

def initial(robots):
    '''
    every robots own its parents and childs
    :param robots:
    :return:
    '''
    robots[2].setParents(0, 1)
    robots[3].setParents(0, 2)
    robots[4].setParents(1, 2)
    robots[5].setParents(2, 3)
    robots[6].setParents(4, 2)
    robots[7].setParents(5, 3)
    robots[8].setParents(4, 6)
    robots[9].setParents(6, 8)

    robots[0].setChild(2)
    robots[0].setChild(3)

    robots[1].setChild(2)
    robots[1].setChild(4)

    robots[2].setChild(4)
    robots[2].setChild(5)

    robots[3].setChild(5)
    robots[3].setChild(7)

    robots[4].setChild(8)
    robots[4].setChild(6)

    robots[5].setChild(7)
    robots[5].setChild(10)

    robots[6].setChild(8)
    robots[6].setChild(9)

    robots[7].setChild(10)

    robots[8].setChild(9)
    robots[8].setChild(10)

    robots[9].setChild(10)

    robots[10].setChild(7)
    robots[10].setChild(8)
    robots[10].setChild(9)


def distance(coord1, coord2):
    return np.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)


def cmp_by_value(lhs):
    return lhs[1]

def cmp_by_value2(lhs):
    return lhs[1]


def set_neighbors(robots):
    robot_num = len(robots)

    center_x = (robots[Beacon_1id].coord[0] + robots[Beacon_3id].coord[0]) / 2
    center_y = (robots[Beacon_1id].coord[1] + robots[Beacon_3id].coord[1]) / 2

    for r in robots:
        r.nei_id = []
        r.nei_pos = []
        r.nei_dis = []
        r.myNeighbor = []
        r.height = [r.coord[0]-center_x, r.coord[1] - center_y]

    for i in range(robot_num):
        for j in range(robot_num):
            dis = distance(robots[i].coord, robots[j].coord)
            if dis < communication_distance and i != j:
                robots[i].nei_id.append(j)
                robots[i].nei_dis.append(dis)
                # robots[i].dic_neighbors[j] = dis
                robots[i].nei_pos.append(robots[j].coord)
                robots[i].myNeighbor.append([j, dis])

    for r in robots:
        r.myNeighbor = sorted(r.myNeighbor, key=cmp_by_value)


def show_old_new(oldpoints, targetpoints):
    global times
    fig = plt.figure(10+times)
    plt.scatter(targetpoints[:, 0], targetpoints[:, 1], c='b')
    plt.scatter(oldpoints[:, 0], oldpoints[:, 1], c='r')
    for i in range(len(oldpoints)):
        plt.annotate(s=i, xy=(oldpoints[i, 0], oldpoints[i, 1]), xytext=(-5, 5), textcoords='offset points')
        plt.annotate(s=i, xy=(targetpoints[i, 0], targetpoints[i, 1]), xytext=(-5, 5), textcoords='offset points')
    plt.plot(20, 20, 'black')
    plt.plot(0, 0, 'black')
    # times = times + 1


def show2d(targetpoints, robots):
    global times
    fig = plt.figure(times)
    # plt.title('epoch %d' % times)
    # plt.scatter(targetpoints[:, 0], targetpoints[:, 1], c='b')
    plotx = []
    ploty = []
    i = 0
    maxdis = 0
    maxid1=0
    maxid2 = 0
    for r in robots:
        plotx.append(r.get_position()[0])
        ploty.append(r.get_position()[1])
        plt.annotate(s=i, xy=(r.get_position()[0], r.get_position()[1]), xytext=(-5, 5), textcoords='offset points')
        # plt.annotate(s=i, xy=(targetpoints[i, 0], targetpoints[i, 1]), xytext=(-5, 5), textcoords='offset points')
        if r.state != 3:
            plt.plot([r.get_position()[0], robots[r.parent1id].coord[0]],
                     [r.get_position()[1], robots[r.parent1id].coord[1]], c='blue')
            if distance(r.coord, robots[r.parent1id].coord) > maxdis:
                maxdis = distance(r.coord, robots[r.parent1id].coord)
                maxid1 = r.id
                maxid2 = r.parent1id
            plt.plot([r.get_position()[0], robots[r.parent2id].coord[0]], [r.get_position()[1], robots[r.parent2id].coord[1]], c='blue')
            if distance(r.coord, robots[r.parent2id].coord) > maxdis:
                maxdis = distance(r.coord, robots[r.parent2id].coord)
                maxid1 = r.id
                maxid2 = r.parent2id
        for child in r.childid:
            plt.plot([r.get_position()[0], robots[child].coord[0]],
                     [r.get_position()[1], robots[child].coord[1]], c='blue')
            if distance(r.coord, robots[child].coord) > maxdis:
                maxdis = distance(r.coord, robots[child].coord)
                maxid1 = r.id
                maxid2 = child
        plt.plot([robots[0].coord[0], robots[1].coord[0]],
                 [robots[0].coord[1], robots[1].coord[1]], c='blue')
        i = (i + 1) % len(robots)
    plt.scatter(plotx, ploty, c='r')
    print('maxdis, maxid1, maxid2', maxdis, maxid1, maxid2)
    mindis = 999
    minid1 = 0
    maxid2 = 0
    for r in robots:
        for j in robots:
            if r.id == j.id:
                continue
            tempdis = distance(r.coord, j.coord)
            if tempdis < mindis:
                mindis = tempdis
                minid1 = r.id
                minid2 = j.id
    print('mindis, minid1, minid2', mindis, minid1, minid2)
    # plt.plot(20, 20, 'black')
    # plt.plot(0, 0, 'black')
    # times = times + 1
    plt.savefig('formation init.eps')


def anim(list_points, points):
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
    # animate2.save('circle_shrink.gif', writer='imagemagick', fps=3000)
    plt.show()


def cannot_move(robots):
    movecount = 0
    for r in robots:
        if r.isMove == False:
            movecount = movecount + 1
    if movecount == len(robots):
        return False
    return True


def circle_shrink(robots):
    logging.debug('come to circle_shrink !!!!!!!!!!!!!!!!!!!!')
    center_x = (robots[Beacon_1id].coord[0] + robots[Beacon_3id].coord[0]) / 2
    center_y = (robots[Beacon_1id].coord[1] + robots[Beacon_3id].coord[1]) / 2
    speed = 0.3
    count = 1

    for i in range(count):
        order_list = []
        for r in robots:
            dis_robot_center = distance([center_x, center_y], r.coord)
            order_list.append([r.id, dis_robot_center])
        order_list = sorted(order_list, key=cmp_by_value2)

        # random_list = [i for i in range(len(robots))]
        # shuffle(random_list)
        # for m in random_list:
        # for m in order_list:
        #     r = robots[m[0]]
        for r in robots:

            direct = math.atan2(center_y - r.coord[1], center_x - r.coord[0])
            # direct = (np.random.random()*2-1)*np.pi
            print('r robot is [%f, %f],center coord is[%f, %f] '% (r.coord[0], r.coord[1], center_x, center_y))
            print('i is %d robot[%d] dircet is %f, speed is %f' % (i, r.id, 180*direct/math.pi, speed * (count - i)/count))
            r.move(speed * (count - i)/count, direct+(np.random.random()*2-1) * np.pi/4)
            set_neighbors(robots)


def point_curve(list_points,points):
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
    # plt.savefig('init_final.pdf')
    plt.show()


def RobotList():
    print('this is robot_listp1')
    print(Robot_listp1.__len__())
    for index in range(len(Robot_listp1)):
        if index != Beacon_1id and index != Beacon_2id and index != Beacon_3id:
            plt.figure(1000+index)
            # plt.title(
            #     'distance between robot[9] and parents \ncollision_distance constraint 0.2 \ncommunication_distance constraint 4.2')
            plt.xlabel('epoch')
            plt.ylabel('distance')
            Robot_Arrayp1 = np.array(Robot_listp1[index])
            plt.plot(Robot_Arrayp1[:, 0], Robot_Arrayp1[:, 1], label='distance with p1')
            Robot_Arrayp2 = np.array(Robot_listp2[index])
            plt.plot(Robot_Arrayp2[:, 0], Robot_Arrayp2[:, 1], label='distance with p2')
            plt.legend()
            strName = index.__str__() + 'parent.pdf'
            # plt.savefig(strName)
    plt.figure(99999)
    # plt.title(
    #     'distance between robot[9] and parents \ncollision_distance constraint 0.2 \ncommunication_distance constraint 4.2')
    plt.xlabel('epoch')
    plt.ylabel('distance')
    Robot2_Arrayp1 = np.array(Robot2_listp1)
    plt.plot(Robot2_Arrayp1[:, 0], Robot2_Arrayp1[:, 1], label='distance with p1')
    Robot2_Arrayp2 = np.array(Robot2_listp2)
    plt.plot(Robot2_Arrayp2[:, 0], Robot2_Arrayp2[:, 1], label='distance with p2')
    plt.legend()

    plt.show()


def debugRobot(robots, r, epoch, points):
    # DEBUG print robot[9]'s state Beacon1
    if r.id == Beacon_1id:
        tempdis = np.sqrt(r.coord[0]**2 + r.coord[1]**2)
        Beacon_1List.append(copy.deepcopy([epoch, tempdis]))

    if r.isBeacon == False:
        distance_with_p1 = np.sqrt(((r.coord[0] - robots[r.parent1id].coord[0]) ** 2 +
                                    (r.coord[1] - robots[r.parent1id].coord[1]) ** 2))
        old_distance_with_p1 = distance(points[r.id, 0:2], points[r.parent1id, 0:2])
        distance_with_p2 = np.sqrt(
            ((r.coord[0] - robots[r.parent2id].coord[0]) ** 2 + (r.coord[1] - robots[r.parent2id].coord[1]) ** 2))
        old_distance_with_p2 = distance(points[r.id, 0:2], points[r.parent2id, 0:2])

        # Robot3_list.append(copy.deepcopy([epoch, distance_with_p1 + distance_with_p2
        #                                   ]))
        Robot_listp1[r.id].append(copy.deepcopy([epoch, distance_with_p1-old_distance_with_p1]))
        Robot_listp2[r.id].append( copy.deepcopy([epoch, distance_with_p2-old_distance_with_p2]))
    if r.id == 4:
        distance_with_p1 = np.sqrt(((r.coord[0] - robots[r.parent1id].coord[0]) ** 2 +
                                    (r.coord[1] - robots[r.parent1id].coord[1]) ** 2))
        distance_with_p2 = np.sqrt(
            ((r.coord[0] - robots[r.parent2id].coord[0]) ** 2 + (r.coord[1] - robots[r.parent2id].coord[1]) ** 2))
        Robot2_listp1.append(copy.deepcopy([epoch, distance_with_p1-old_distance_with_p1]))
        Robot2_listp2.append(copy.deepcopy([epoch, distance_with_p2-old_distance_with_p2]))





def formation_control_manyTimes(robots, points):
    epochs = 500
    global times, AdjustTime
    velocity = 0.1
    deltaH = 0.7
    list_V = np.arange(0.1,0.5,0.01)
    adjusttimelist = []
    epochlist = []
    for r in list_V:
        velocity = r
        AdjustTime = 0
        list_points = []
        initial(robots)
        for i in range(len(points)):
            robots[i].set_position([points[i][0], points[i][1]])
        set_initial_height(robots)
        set_neighbors(robots)
        times = 0

        for epoch in range(epochs):
            times = times + 1
            # if times == 120 or times == 121 or times == 122:
            #     targetpoints = []
            #     show2d(np.array(targetpoints), robots)
            print('this is %d picture' % times)
            targetpoints = []
            oldpoints = []

            for r in robots:
                oldpoints.append(r.coord)
                debugRobot(robots, r, epoch, points)
            list_points.append(copy.deepcopy(oldpoints))
            # move beacon
            adjust_height(robots, deltaH, velocity, np.pi / 4)
            for r in robots:
                if r.isBeacon == True:
                    tp = r.move(velocity, math.pi / 4)
                    targetpoints.append(tp)
                    set_neighbors(robots)
            # te_calculate_parents(robots)



            oldpoints = []
            for r in robots:
                oldpoints.append(r.coord)
                debugRobot(robots, r, epoch, points)
            list_points.append(copy.deepcopy(oldpoints))

            # move non-beacon
            adjust_height(robots, deltaH, velocity, np.pi / 4)
            for r in robots:
                if r.isBeacon == False:
                    tp = r.move(velocity, math.pi / 4)
                    targetpoints.append(tp)
                    set_neighbors(robots)

            # if distance(robots[Beacon_3id].coord,points[Beacon_3id,:]+30/np.sqrt(2)) < (deltaH):
            if robots[Beacon_3id].coord[0]>points[Beacon_3id, 0] + 30/np.sqrt(2) and \
                    robots[Beacon_3id].coord[1]>points[Beacon_3id,1] + 30/np.sqrt(2):
                adjusttimelist.append(AdjustTime)
                epochlist.append(epoch)
                print('you get the finish condition adjustionlist epochlist is',adjusttimelist,epochlist)
                break

    FinalList = np.array([list_V,adjusttimelist,epochlist]).reshape(3,(len(list_V)))
    print('FinalLIst is ', FinalList)
    np.savetxt('adjustList.npy', FinalList)

    anim(list_points, points)
    point_curve(list_points, points)
    print('AdjustTime is', AdjustTime)
    print('epoch times is', epoch)
    plt.show()


def formation_control2(robots, points):
    epochs = 500
    global times, AdjustTime
    velocity = 0.3
    deltaH = 0.7
    list_points = []
    for epoch in range(epochs):
        times = times + 1
        # if times == 120 or times == 121 or times == 122:
        #     targetpoints = []
        #     show2d(np.array(targetpoints), robots)
        print('this is %d picture' % times)
        targetpoints = []
        oldpoints = []

        for r in robots:
            oldpoints.append(r.coord)
            debugRobot(robots, r, epoch, points)
        list_points.append(copy.deepcopy(oldpoints))
        # move beacon
        adjust_height(robots, deltaH, velocity, np.pi / 4)
        for r in robots:
            if r.isBeacon == True:
                tp = r.move(velocity, math.pi / 4)
                targetpoints.append(tp)
                set_neighbors(robots)

        oldpoints = []
        for r in robots:
            oldpoints.append(r.coord)
            debugRobot(robots, r, epoch, points)
        list_points.append(copy.deepcopy(oldpoints))

        # move non-beacon
        adjust_height(robots, deltaH, velocity, np.pi / 4)
        for r in robots:
            if r.isBeacon == False:
                tp = r.move(velocity, math.pi / 4)
                targetpoints.append(tp)
                set_neighbors(robots)

        # if distance(robots[Beacon_3id].coord,points[Beacon_3id,:]+30/np.sqrt(2)) < (deltaH):
        if robots[Beacon_3id].coord[0]>points[Beacon_3id, 0] + 30/np.sqrt(2) and \
                robots[Beacon_3id].coord[1]>points[Beacon_3id,1] + 30/np.sqrt(2):
            break

    anim(list_points, points)
    point_curve(list_points, points)
    print('AdjustTime is', AdjustTime)
    print('epoch times is', epoch)
    plt.show()


robot_Num = 0
beacon_Num = 0
communication_distance = 0
Beacon_1id = 0
Beacon_2id = 1
Beacon_3id = 10


def set_initial_height(robots):
    center_x = (robots[Beacon_1id].real_position[0] + robots[Beacon_3id].real_position[0]) / 2
    center_y = (robots[Beacon_1id].real_position[1] + robots[Beacon_3id].real_position[1]) / 2
    for r in robots:
        r.initial_height = [r.coord[0]-center_x, r.coord[1] - center_y]

def initial_and_measured_neighbor(robots):
    global communication_distance
    '''
    use the current localization information modify the robots' neighbor information
    :param points: 
    :param robots: 
    :return: 
    '''
    center_x = (robots[Beacon_1id].real_position[0] + robots[Beacon_3id].real_position[0]) / 2
    center_y = (robots[Beacon_1id].real_position[1] + robots[Beacon_3id].real_position[1]) / 2

    for r in robots:
        r.isFinalPos = False
        r.nei_id = []
        r.measured_distance = {}
        r.myNeighbor = []
        r.height = [r.coord[0]-center_x, r.coord[1] - center_y]

    for i in range(robot_Num):
        for j in range(i+1, robot_Num):
            np.random.seed(12345)
            tempDistance = np.sqrt((robots[i].get_real_position()[0] - robots[j].get_real_position()[0]) ** 2
                                   + (robots[i].get_real_position()[1] - robots[j].get_real_position()[1]) ** 2)
            # tempDistance = tempDistance + tempDistance * (np.random.random() * 0.02 - 0.01)  # 是否加噪声
            if tempDistance < communication_distance:
                robots[i].myNeighbor.append([j, tempDistance])
                robots[j].myNeighbor.append([i, tempDistance])
    for r in robots:
        r.myNeighbor = sorted(r.myNeighbor, key=cmp_by_value)
        r.nei_id = []
        for nei in r.myNeighbor:
            nid = nei[0]
            r.nei_id.append(nid)
            r.measured_distance[nid] = nei[1]


def create_formation_topology():
    np.random.seed(6)
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
        # robots[i].set_coord([points[i][0], points[i][1]])
        robots[i].set_real_position([points[i][0], points[i][1]])

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


def adjust_height(robots, delta_H, velocity, direct):
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
                r.move(dis, direct)
                break

def main():
    sess = tf.Session()
    psolver = PositionSolver(sess, 50, 0.02)
    robots = create_formation_topology()
    set_initial_height(robots)
    initial_and_measured_neighbor(robots)

    # GROLO_LOCALIZATION
    setInitial_by_dvdistance(robots)
    localization_gradient_descent(robots, psolver, epochs=5)
    GROLO_position = localization_GROLO(robots, robot_Num - beacon_Num-1)
    print('grolo position', GROLO_position)

    # formation control
    epochs = 500
    global times, AdjustTime
    velocity = 0.3
    deltaH = 0.7
    for epoch in range(epochs):
        times = times + 1
        print('this is %d picture' % times)

        # move beacon
        adjust_height(robots, deltaH, velocity, np.pi / 4)
        for r in robots:
            if r.isBeacon == True:
                tp = r.move(velocity, math.pi / 4)
                set_neighbors(robots)

        # move non-beacon
        adjust_height(robots, deltaH, velocity, np.pi / 4)
        for r in robots:
            if r.isBeacon == False:
                tp = r.move(velocity, math.pi / 4)
                set_neighbors(robots)


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





