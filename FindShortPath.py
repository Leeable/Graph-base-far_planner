import GraphUtil

if __name__ == "__main__":
    # 实例化类对象
    graph_util = GraphUtil.GraphUtil()
    # 方便测试，直接用graph上的node转坐标
    start = GraphUtil.id2pos(6)
    end = GraphUtil.id2pos(251)
    # print(GraphUtil.neighbor_node(525))
    global_path = GraphUtil.find_path(start, end)
    print("path=", global_path)
    GraphUtil.draw_path(global_path)
