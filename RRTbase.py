import math 
import random
import pygame

class RRTMap:

    def __init__(self, start, goal, dimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.dimensions = dimensions
        self.width, self.height = dimensions

        # Window
        self.windowName = 'RRT Path Planning'
        pygame.display.set_caption(self.windowName)
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.fill((255,255,255))
        self.nodeRad = 2 # Node Radius
        self.nodeThickness = 0 # Node Thickness
        self.edgeThickness = 1 # Edge Thickness

        # Obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        # Colors
        self.grey = (70, 70, 70)
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.orange = (255, 165, 0)


    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.red, self.start, self.nodeRad + 7, 0) # Draw Start
        pygame.draw.circle(self.map, self.green, self.goal, self.nodeRad + 20, 5) # Draw Goal

        self.drawObs(obstacles) 

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, self.nodeRad + 3, self.nodeThickness)
            for i in range(0, len(path) - 1):
                pygame.draw.line(self.map, self.blue, path[i], path[i+1], self.edgeThickness + 5)

    def drawObs(self, obstacles):
        obs = obstacles.copy()

        while(len(obs) > 0):
            obstacle = obs.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRTGraph:

    def __init__(self, start, goal, dimensions, obsnum, obsdim):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.dimensions = dimensions
        self.width, self.height = dimensions

        # Tree
        self.X = []
        self.Y = []
        self.parent = []

        self.X.append(x)
        self.Y.append(y)
        self.parent.append(0)

        # Obstacles
        self.obstacles = []
        self.obsdim = obsdim    
        self.obsnum = obsnum

        # Path
        self.goalstate = None
        self.path = []


    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.width - self.obsdim))
        uppercornery = int(random.uniform(0, self.height - self.obsdim))

        return (uppercornerx, uppercornery)

    def makeobs(self):
        obs = []

        for i in range(self.obsnum):
            rectangle = None
            startGoalCol = True
            
            while(startGoalCol):
                upperCorner = self.makeRandomRect()
                rectangle = pygame.Rect(upperCorner, (self.obsdim, self.obsdim))

                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    startGoalCol = True
                else:
                    startGoalCol = False

            obs.append(rectangle)

        self.obstacles = obs
        return obs;
               

    def add_node(self, n, x, y):
        self.X.insert(n, x)
        self.Y.insert(n, y)

    def remove_node(self, n):
        self.X.pop(n)
        self.Y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n);

    def number_of_nodes(self):
        return len(self.X)

    def distance(self, id1, id2): # id1 and id2 are the indices of the nodes in the tree
        (x1, y1) = (self.X[id1], self.Y[id1])
        (x2, y2) = (self.X[id2], self.Y[id2])

        return (float((x1 - x2) ** 2) + float((y1 - y2) ** 2)) ** 0.5

    def sample_env(self):
        x = int(random.uniform(0, self.width))
        y = int(random.uniform(0, self.height))

        return x, y

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.X[n], self.Y[n])

        for rect in self.obstacles:
            if rect.collidepoint(x, y):
                self.remove_node(n)
                return False

        return True

    def crossObstacle(self, x1, y1, x2, y2):
        obs = self.obstacles.copy()

        while(len(obs) > 0):
            obstacle = obs.pop(0)
            
            # Interpolation to check if the edge crosses an edge
            for i in range(101): 

                u = i / 100;
                x = x1*u + x2*(1 - u)
                y = y1*u + y2*(1 - u)

                if obstacle.collidepoint(x, y):
                    return True
        
        return False

    # Connecting nodes
    def connect(self, n1, n2):
        (x1, y1) = (self.X[n1], self.Y[n1])
        (x2, y2) = (self.X[n2], self.Y[n2])

        if(self.crossObstacle(x1, y1, x2, y2)):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def nearest(self, n):
        dmin = self.distance(0, n)
        near_id = 0

        for i in range(1, len(self.X)):
            d = self.distance(n, i)
            if d < dmin and n != i:
                dmin = d
                near_id = i

        return near_id

    def step(self, near_id, n_rand, dmax = 25):
        d = self.distance(near_id, n_rand)
        if d > dmax:
            u = dmax/d
            (x1, y1) = (self.X[near_id], self.Y[near_id])
            (x2, y2) = (self.X[n_rand], self.Y[n_rand])
            (px, py) = (x2 - x1, y2 - y1)
            theta = math.atan2(py, px)
            (x, y) = (int(x1 + dmax*math.cos(theta)), int(y1 + dmax*math.sin(theta)))

            self.remove_node(n_rand)

            if(abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax):
                self.goalFlag = True
                self.goalstate = n_rand
                self.add_node(n_rand, self.goal[0], self.goal[1])

            else:
                self.add_node(n_rand, x, y)

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        n_near = self.nearest(n)
        self.step(n_near, n)
        self.connect(n_near, n)
        return self.X, self.Y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_env()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.X, self.Y, self.parent


    def path_to_goal(self):
        if(self.goalFlag):
            self.path = []
            self.path.append(self.goalstate)
            parent = self.parent[self.goalstate]

            while(parent != 0):
                self.path.append(parent)
                parent = self.parent[parent]

            self.path.append(0)

            return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            (x, y) = (self.X[node], self.Y[node])
            pathCoords.append((x, y))

        return pathCoords

    def cost(self): pass