import sys
sys.path.append('..')
import ReedsSheppPath.reeds_shepp_path_planning as rs_path
from collections import namedtuple
from scipy.spatial import KDTree
from queue import PriorityQueue

Node = namedtuple('Node', 'xind yind yawind direction x y yaw directions steer cost pind')
Config = namedtuple('Config', 'minx miny minyaw maxx maxy maxyaw xw yw xyreso yawreso')

class Path:
    def __init__(self):
        self.x = []  # type=float
        self.y = []  # type=float
        self.yaw = []  # type=float
        self.directions = []  # type=int
        self.total_cost = None  #type=float

def calc_hybrid_astar_path(sx, sy, syaw, gx, gy, gyaw, ox, oy, xyreso, yawreso):
    syaw, gyaw = rs_path.pi_2_pi(syaw), rs_path.pi_2_pi(gyaw)
    assert len(ox) == len(oy)
    ox = np.array(ox).reshape((len(ox),)).astype('float')
    oy = np.array(oy).reshape((len(oy),)).astype('float')
    kdtree = KDTree(np.hstack([ox, oy]))
    c = calc_config(ox, oy, xyreso, yawreso)
    start_node = Node(xind=round(sx/xyreso),
                      yind=round(sy/xyreso),
                      direction=True,
                      x=float(sx),
                      y=float(sy),
                      yaw=syaw,
                      directions=[True],
                      steer=0.0,
                      cost=0.0,
                      pind=-1)
    goal_node = Node(xind=round(gx/xyreso),
                     yind=round(gy/xyreso),
                     direction=True,
                     x=float(gx),
                     y=float(gy),
                     yaw=float(syaw),
                     directions=[True],
                     steer=0.0,
                     cost=0.0,
                     pind=-1)

    h_dp = calc_holonomic_with_obstacle_heuristic(goal_node, ox, oy, xyreso)

    _open, closed = {}, {}
    fnode = None
    _open[calc_index(start_node, c)] = start_node
    pq = PriorityQueue()
    pq.put(calc_index(start_node, c), calc_cost(start_node, h_dp, ngoal, c))

    u, d = calc_motion_inputs()
    nmotion = len(u)

    while True:
        # TODO: do stuff
        break

    print('number of expanded nodes: {expand}'.format(len(_open) + len(closed)))
    path = get_final_path(closed, fnode, start_node, c)
    return path

def calc_config(ox, oy, xyreso, yawreso):
    pass

def calc_holonomic_with_obstacle_heuristic(goal_node, ox, oy, xyreso):
    pass

def calc_index(node, config):
    pass

def calc_cost(node, h_dp, goal_node, config):
    pass

def get_final_path(closed_nodes, fnode, start_node, config):
    pass
