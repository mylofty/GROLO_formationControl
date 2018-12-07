import math
import numpy as np
from config import *
from matplotlib import pyplot as plt
from triangle_extension_file import triangle_extension


class Robot(object):
    def __init__(self, id):
        self.id = id
        self.isBeacon = False
        self.isFinalPos = False
        self.coord = []  # robot's coord [x, y]
        self.nei_id = []
        self.myNeighbor = []  # this is construct like [id, distance]
        self.measured_distance = {} # this is a map,  key is neighbor's id, value is neighbor's distance
        self.loss_dump = []  # loss curve

        # triangle extension
        self.state = 0
        self.parent1 = -1
        self.parent2 = -1
        self.root1 = -1
        self.root2 = -1
        self.extra = -1
        self.query1 = -1
        self.query2 = -1

        # formation control
        self.nei_pos = []
        self.initial_height = []
        self.height = []  # distance to the center point
        self.childid = []
        self.isMove = False

    def set_parents(self, p1, p2):
        self.parent1 = p1
        self.parent2 = p2

    def set_child(self, child_id):
        self.childid.append(child_id)

    def set_beacon(self):
        self.isBeacon = True
        self.state = 3
        self.root1 = self.root2 = self.id

    def get_coord(self):
        return self.coord

    def distance(self, coord1, coord2):
        return np.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

    def set_coord(self, coord):
        self.coord = coord

    def distance_to(self, rid):
        for nei in self.myNeighbor:
            if nei[0]==rid:
                return nei[1]

    def triangle_extension(self, probot):
        triangle_extension(self, probot)

    def run(self, psolver, neighbors, dists):
        if(self.isBeacon == True):
            return
        if neighbors is None or neighbors == []:
            return
        # print('neighbor ', neighbors)
        # print('distss ', dists)
        # print('origin coord', self.coord)
        coord, loss = psolver.solver(self.coord, neighbors, dists)
        # print('loss is ', loss)
        assert not math.isnan(loss)
        if not math.isnan(coord[0]):
            self.set_coord(coord)
            self.loss_dump.append(loss)

    def move(self, speed, direct):
        self.isMove = False

        step = 1
        s_error = speed * deltaV
        d_error = deltaD
        # target_x = self.coord[0] + step * speed * np.cos(direct)
        # target_y = self.coord[1] + step * speed * np.sin(direct)
        # speed = speed + np.random.random() * s_error * 2 - s_error
        # direct = direct + np.random.random() * d_error * 2 - d_error
        length = step * speed
        dx = length * np.cos(direct)
        dy = length * np.sin(direct)

        max_dx = step * (speed + s_error) * np.cos(direct)
        max_dy = step * (speed + s_error) * np.sin(direct)
        min_dx = step * (speed - s_error) * np.cos(direct)
        min_dy = step * (speed - s_error) * np.sin(direct)

        dx_list = []
        dy_list = []
        for x in range(10):
            dx_list.append(min_dx + (max_dx - min_dx) * x / 10)
            dy_list.append(min_dy + (max_dy - min_dy) * x / 10)

        # when the robot can move
        if self.id == 3:
            print('self.nei_id is', self.nei_id)
            print('self.nei_pos is', self.nei_pos)



        for index in range(len(self.nei_id)):
            for d in range(len(dx_list)):
                if self.distance(self.nei_pos[index], np.array(self.coord) + np.array([dx_list[d], dy_list[d]])) < collision_distance:
                    print('robot[{}] can not moving less than collision with robot[{}]'.format(self.id, self.nei_id[index]))
                    return False

            # when neighbor is robot's parents and child. cannot over than communication_distance
            if(self.nei_id[index] == self.parent1 or self.nei_id[index] == self.parent2
                              or (self.nei_id[index] in self.childid)):
                for d in range(len(dx_list)):
                    if self.distance(self.nei_pos[index],
                                     np.array(self.coord) + np.array([dx_list[d], dy_list[d]])) > allow_distance:
                        print('robot[{}] can not moving more than allow_distance with robot[{}]'.format(self.id, self.nei_id[index]))
                        return False
        self.coord = [self.coord[0]+dx, self.coord[1]+dy]
        self.isMove = True
        # return target position
        return True

    def show_loss_curve(self):
        plt.figure(10)
        print('loss_dump is', self.loss_dump)
        length = len(self.loss_dump)
        print('curve length is ', length)
        plt.annotate(s=round(self.loss_dump[length-1], 2), xy=((length-1)*self.epoch, self.loss_dump[length-1]), xytext=(-5, 5),
                     textcoords='offset points')
        # plt.annotate(s=round(self.loss_dump[length - 2], 2), xy=((length - 2)*self.epoch, self.loss_dump[length - 2]), xytext=(-5, 5),
        #              textcoords='offset points')
        plt.plot(np.arange(0,length,step=1)*self.epoch, self.loss_dump)
        np.savetxt('./loss_dump2.txt',np.array(self.loss_dump))
        plt.show()
