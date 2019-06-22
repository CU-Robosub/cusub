from tasks.start_gate import Attack
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    left_side = True
    p = Pose()
    p.position.x = 0
    p.position.y = 0
    p2 = Pose()
    p2.position.x = 2
    p2.position.y = 0
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 2
    p2.position.y = 2
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 0
    p2.position.y = 2
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = -2
    p2.position.y = 2
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = -2
    p2.position.y = 0
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = -2
    p2.position.y = -2
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 0
    p2.position.y = -2
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 2
    p2.position.y = -2
    p3 = Attack.adjust_gate_pose(p, p2, dist_behind=1, small_leg_left_side=left_side, third_leg_adjustment=1)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()

    print('\n\n')
    print('sub\n----')
    print(p.position)
    print('\ngate\n----')
    print(p2.position)
    print('\ngoal\n----')
    print(p3.position)