

class my_planner(object):
    def __init__(self):
        pass
    
    def get_waypoint(self):
        '''
        Get the current best next waypoint
        '''

        # use RRT with fast approximate collision check
        # call moveit RRT library with 1 sec timeout
        RRT.solve()
        # from nearby points, get the best waypoint
        tree = RRT.getPlannerInfo()
        for vertex in tree:
            if euc_dist(vertex, cur_robot_pos) < W:
                vertex_val = self.heuristic_vertex_value(vertex)
                if vertex_val > best_vertex_value:
                    best_vertex = vertex
                    best_vertex_value = vertex_val
        
        return best_vertex

    
    def heuristic_vertex_value(self, vertex):
        # value_vertex = min cost path from source to vertex + 
        vertex_val = dist(source, vertex) + dist(vertex, goal)
        return vertex_val

