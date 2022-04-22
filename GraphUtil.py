import networkx as nx
import math
import BaseUtils
import matplotlib.pyplot as plt
from queue import PriorityQueue


def graph_net(global_graph_edge=[]):
    g = nx.Graph()
    for node_id in global_graph_edge:
        # Calculating the distances between the nodes as edge's weight.
        g.add_edge(node_id[0], node_id[1])  # 根据自己的需要为节点添加边
    return g


class GraphUtil:
    # generate normal graph through matterport.vgh
    global_graph = BaseUtils.Openvgh("matterport.vgh")
    graph_edge = BaseUtils.Edge(global_graph)
    # networkx generate
    global_graph_net = graph_net(graph_edge)
    node_id_list = list(global_graph_net.nodes.keys())
    nx_pos = BaseUtils.NodePos_nx(global_graph)
    id_table = BaseUtils.Id_list(global_graph)


def node_search(node_id, radius):
    id_list = GraphUtil.node_id_list
    close_set = []
    for count in range(0, len(id_list)):
        id = id_list[count]
        node1 = GraphUtil.nx_pos.get(int(node_id))
        node2 = GraphUtil.nx_pos.get(int(id))
        dist = distance(node1, node2)
        if (dist < radius) & (int(id) != int(node_id)):
            # temp = dist
            close_set.append(id)
    nearnode = close_set
    return nearnode


def valid_node(to_node_id, from_node_id):
    # TODO 现在的surf不靠谱，我要重新算一次
    # surf = surf_dir(from_node_id)
    neighbour = neighbor_node(from_node_id)
    front_from_pos = id2pos(neighbour[0])
    back_from_pos = id2pos(neighbour[1])

    from_pos = id2pos(from_node_id)

    surf1 = norm(from_pos, front_from_pos)
    surf2 = norm(from_pos, back_from_pos)
    surf = (surf1, surf2)
    to_pos = id2pos(to_node_id)
    # from到to的向量
    norm_dir = norm(from_pos, to_pos)
    # 角平分线
    temp_surf = add(surf[0], surf[1])
    topo_surf = divide(temp_surf, math.sqrt((temp_surf[0]) ** 2 + (temp_surf[1]) ** 2))
    dot_value1 = mult(surf[1], topo_surf)
    dot_value2 = mult(topo_surf, surf[0])
    # 得到node的类型，0为UNKNOWN 1为CONVEX 2为CONCAVE 3为PILLAR
    direct = free_direct(from_node_id)
    valid = 0  # 标识量
    if direct == 0:
        valid = 0
    if direct == 1:
        val1 = mult(norm_dir, topo_surf)
        val2 = mult(norm_dir, topo_surf)
        if val1 <= dot_value1 or val2 <= dot_value2:
            valid = 1
    if direct == 2:
        val1 = mult(norm_dir, topo_surf)
        val2 = mult(norm_dir, topo_surf)
        if val1 >= dot_value1 or val2 >= dot_value2:
            valid = 1
    if direct == 3:
        valid = 0
    return valid


def neighbor_node(node_id):
    neighbor = GraphUtil.global_graph_net.neighbors(node_id)
    table = []
    for n in neighbor:
        table.append(n)
    return table


def h(current, goal):
    current_pos = id2pos(current)
    goal_pos = id2pos(goal)
    (x1, y1) = current_pos
    (x2, y2) = goal_pos
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return math.sqrt(dx * dx + dy * dy)


def free_direct(node_id):
    id_table = GraphUtil.id_table
    num = id_table.index(str(node_id))
    node_direct = GraphUtil.global_graph[num].see_direct().get(str(node_id))
    return node_direct


def surf_dir(node_id):
    id_table = GraphUtil.id_table
    num = id_table.index(str(node_id))
    front_dir = GraphUtil.global_graph[num].see_surf_dir_front().get(node_id)
    back_dir = GraphUtil.global_graph[num].see_surf_dir_back().get(node_id)
    node_dir = (front_dir, back_dir)
    return node_dir


def draw_path(global_path):
    GraphUtil.global_graph_net.add_nodes_from(global_path)
    path_edge = []
    for i in range(0, len(global_path) - 1):
        path_edge.append((global_path[i], global_path[i + 1]))
    nx_pos = BaseUtils.NodePos_nx(GraphUtil.global_graph)
    nx.draw(
        GraphUtil.global_graph_net,
        pos=nx_pos,
        node_size=100,
        with_labels=True,
        font_size=10,
    )
    nx.draw_networkx_nodes(
        GraphUtil.global_graph_net,
        nodelist=global_path,
        node_color="g",
        pos=nx_pos,
        node_size=100,
    )
    nx.draw_networkx_edges(
        GraphUtil.global_graph_net,
        pos=nx_pos,
        edgelist=path_edge,
        # edge_cmap=plt.get_cmap('Set2'),
        width=8,
        alpha=0.8,
        # style='-.',
        edge_color="tab:red",
    )
    plt.show()


# 按条件过滤，得到可以直接到达的node
def renew_valid_nodes(cur_node):
    radius = 9
    node_list = node_search(cur_node, radius)  # 存放node周围一定距离的nodes
    while 1:
        if len(node_list) < 10 and radius < 11:
            radius += 0.2
            node_list = node_search(cur_node, radius)
        else:
            break
    # 判断cur_node到node_list里的nodes的有效性
    # print(node_list)
    temp_valid_list = []
    node_valid_list = []
    for node_id in node_list:
        valid = valid_node(node_id, cur_node)  # 从向量层面判断
        if valid == 1:
            temp_valid_list.append(node_id)
        if node_id in neighbor_node(cur_node) and node_id not in temp_valid_list:
            temp_valid_list.append(node_id)
    # temp_valid_list = node_list

    for val_node in temp_valid_list:
        val_count = 0  # 判断相交有效性
        global_node_id_list = list(GraphUtil.global_graph_net.nodes.keys())
        # global_node_id_list = node_list
        global_node_id_list.remove(cur_node)
        global_node_id_list.remove(val_node)
        for nodes in global_node_id_list:
            neighbor_list = neighbor_node(nodes)  # 取出graph里的每个node的neighbor
            for neighbor in neighbor_list:
                if neighbor == cur_node or neighbor == val_node:
                    continue
                cur = id2pos(cur_node)  # 当前的cur_node
                done = id2pos(val_node)  # node_list里潜在的node
                center = id2pos(nodes)  # 当前graph遍历到的点
                neighbors = id2pos(neighbor)  # 当前遍历到的点的neighbor
                val = dointersect(cur, done, center, neighbors)
                val_count += val
        if val_count == 0:
            node_valid_list.append(val_node)
    # print(node_valid_list)
    return node_valid_list


def find_path(start_node=(), goal_node=()):
    # graph start node
    near_start = near_node(start_node)
    print("near_start:", near_start)
    near_end = near_node(goal_node)
    print("near_end:", near_end)
    # TODO  用到达的node取它的edge的node，做向量，判断下一个目标点是不是和该向量的夹角冲突，如果冲突了，就要回撤
    open_set = PriorityQueue()  # 写反了
    open_set.put([0, near_start])  # 将起点加入open_set
    close_set = {near_start: 0}  # 顺便记录节点距离起点的代价
    came_from = {near_start: None}  # 记录父节点
    # 开始循环
    print("Processing please wait")
    global_path = []
    while open_set.empty() != 1:
        current = open_set.get()[1]
        node_valid_list = renew_valid_nodes(current)
        # 最快找到路径
        # if near_end in node_valid_list
        # 找最短路径
        if near_end == current:
            global_path.append(near_end)
            node_id = came_from[current]
            while node_id:
                global_path.append(node_id)
                node_id = came_from[node_id]
            global_path.reverse()
            return global_path
        for next_one in node_valid_list:
            cur_pos = id2pos(current)
            next_pos = id2pos(next_one)
            new_cost = (close_set[current] + distance(cur_pos, next_pos))
            if next_one not in close_set or new_cost < close_set[next_one]:
                close_set[next_one] = new_cost
                priority = new_cost + h(next_one, near_end)
                open_set.put([priority, next_one])
                came_from[next_one] = current


def id2pos(node_id):
    node_pos = GraphUtil.nx_pos.get(int(node_id))
    return node_pos


def near_node(point=()):
    # 把图G里的有用到的node的id变成list
    id_list = GraphUtil.node_id_list
    # 从我的graph里获取位置字典{node_id:(x,y)}
    temp = 5
    close_set = []
    # 循环取出node_id
    for count in range(0, len(id_list)):
        id = id_list[count]
        # 获取该node的位置
        node1 = GraphUtil.nx_pos.get(int(id))
        # 计算距离
        dist = distance(point, node1)
        if dist < temp:
            temp = dist
            close_set.append(id)
    # 返回最后一个距离最小的
    near_node = close_set.pop()
    return near_node

    # 判断下一个点to_node和当前的from_node是不是有效的（即不能穿墙）


def distance(node1=(), node2=()):
    (x1, y1) = node1
    (x2, y2) = node2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


# 由from指向to的归一化向量
def norm(from_node=(), to_node=()):
    dist = distance(from_node, to_node)
    norm_dir = divide(sub(from_node, to_node), dist)
    return norm_dir


# 定义向量除以数
def divide(vector=(), number=1):
    x = vector[0] / number
    y = vector[1] / number
    vector_done = (x, y)
    return vector_done


# 定义向量相减
def sub(from_node=(), to_node=()):
    x = to_node[0] - from_node[0]
    y = to_node[1] - from_node[1]
    node_dir = (x, y)
    return node_dir


def add(from_node=(), to_node=()):
    x = to_node[0] + from_node[0]
    y = to_node[1] + from_node[1]
    node_dir = (x, y)
    return node_dir


# 定义向量点积
def mult(vec1=(), vec2=()):
    value = vec1[0] * vec2[0] + vec1[1] * vec2[1]
    return value


def value2key(value, check_dict={}):
    value_table = list(check_dict.values())
    key_table = list(check_dict.keys())
    num = value_table.index(value)
    key = key_table[num]
    return key


def orientation(node1=(), node2=(), noder=()):
    (x1, y1) = node1
    (x2, y2) = node2
    (x3, y3) = noder
    val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2)
    if abs(val) < 1e-7:
        return 0  # colinear
    if val > 0:
        return 1
    else:
        return 2


# 检查node2是否在线段(node1,node3)
def onsegment(node1, node2, node3):
    (x1, y1) = node1
    (x2, y2) = node2
    (x3, y3) = node3
    if max(x1, x3) >= x2 >= min(x1, x3) and max(y1, y3) >= y2 >= min(y1, y3):
        return True
    else:
        return False


# 判断线段（node1，node2）和线段(node3,node4)相交情况
def dointersect(node1=(), node2=(), node3=(), node4=()):
    o1 = orientation(node1, node2, node3)
    o2 = orientation(node1, node2, node4)
    o3 = orientation(node3, node4, node1)
    o4 = orientation(node3, node4, node2)

    if o1 != o2 and o3 != o4:
        return 1
    if o1 == 0 and onsegment(node1, node3, node2):
        return 1
    if o2 == 0 and onsegment(node1, node4, node2):
        return 1
    if o3 == 0 and onsegment(node3, node1, node4):
        return 1
    if o4 == 0 and onsegment(node3, node2, node4):
        return 1
    else:
        return 0

# if __name__ == '__main__':
#     # 实例化类对象
#     graph_util = GraphUtil()
#     start = id2pos(288)
#     end = id2pos(452)
#     path = find_path(start, end)
#
#     # print("path=", path)
#
#     draw_path(path)
