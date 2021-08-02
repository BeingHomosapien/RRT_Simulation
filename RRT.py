import pygame

from RRTbase import RRTMap
from RRTbase import RRTGraph

def main():
    dimensions = (800, 800)
    start = (50, 50)
    goal = (450, 450)
    obsdim = 25
    obsnum = 50
    max_iterations = 1000

    pygame.init()

    rrt_map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    rrt_graph = RRTGraph(start, goal, dimensions, obsdim, obsnum)

    obstacles = rrt_graph.makeobs()
    rrt_map.drawMap(obstacles)

    # Testing node formation
    iterations = 0
    while(not rrt_graph.path_to_goal() and iterations < max_iterations):
        if iterations%10 == 0: 
            X, Y, parent = rrt_graph.bias(goal)       
            pygame.draw.circle(rrt_map.map, rrt_map.blue, (X[-1], Y[-1]), rrt_map.nodeRad + 2, 0)
            pygame.draw.line(rrt_map.map, rrt_map.orange, (X[-1], Y[-1]), (X[parent[-1]], Y[parent[-1]]), rrt_map.edgeThickness+2)

        else:
            X, Y, parent = rrt_graph.expand()
            pygame.draw.circle(rrt_map.map, rrt_map.blue, (X[-1], Y[-1]), rrt_map.nodeRad + 2, 0)
            pygame.draw.line(rrt_map.map, rrt_map.orange, (X[-1], Y[-1]), (X[parent[-1]], Y[parent[-1]]), rrt_map.edgeThickness+2)

        if iterations%5 == 0:
            pygame.display.update()

        iterations = iterations + 1

    rrt_map.drawPath(rrt_graph.getPathCoords())

    # print(rrt_graph.parent)
    # print(rrt_graph.X)

    pygame.display.update()
    pygame.image.save(rrt_map.map, "RRT_algo.png")
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
    main()
