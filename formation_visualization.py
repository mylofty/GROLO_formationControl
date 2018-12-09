import matplotlib.pyplot as plt
import numpy as np
import os

Folder = "error_5"

Time = 1
def point_curve(list_points, points, curvename):
    global Time
    Time = Time + 1
    plt.figure(Time)
    # plt.title('collision_distance constraint 1.1 \ncommunication_distance constraint 4.2\ndistance between robot[0]\'s position and coordinate origin')
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    list_points = np.array(list_points)
    plt.scatter(points[:, 0], points[:, 1], c='r',label='initial '+curvename) # position coordinate
    plt.scatter(points[:, 0] + 150/np.sqrt(2), points[:, 1] + 150/np.sqrt(2), c='r', marker='+', label='excepted '+curvename)
    plt.scatter(list_points[len(list_points)-1, :, 0], list_points[len(list_points)-1, :, 1], c='', edgecolors='b', marker='o', label='final '+curvename)
    print('list_points',list_points)
    for index in range(11):
        plt.plot(list_points[:, index, 0], list_points[:, index, 1])
    plt.legend()
    plt.savefig(curvename + 'init_final.pdf')
    plt.savefig(curvename + 'init_final.eps')




def main():
    list_coords = np.load(os.path.join(Folder, " percent_list_coords.npy"))
    list_points = np.load(os.path.join(Folder, " percent_list_points.npy"))
    print('list_coord shape ', list_coords.shape)
    point_curve(list_coords[0:498] * 5, list_points[0] * 5, "coordinate")
    point_curve(list_points[0:498] * 5, list_points[0] * 5, "position")
    plt.show()


if __name__ == '__main__':
    main()