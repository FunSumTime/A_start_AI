import heapq
from itertools import count

from dataclasses import dataclass

"""This is my program For Ai that includes the A* algorthim, its like dikjtras algorthim but it includes a neat thing called
    heuristics which is a thing the algorthim uses to see if its going in the rihgt place. the huristc takes the state its in and the cost of how we got there
    then can add logic to say how far is this state from the goal? but its a estimate and its better to underestamite then over as we are trying to find
    the shortest path with this algorthim.
    
    This algorthim was implemented on the 8 puzzel game, where you have a 3 by 3 grid where each index is filled by a tile with a value
    execpt one and that is the zero which will be used to shift the other tiles around, in which you will do to work towards a goal of all the 
    values in a specifc place on the grid
    
    The Actions you can do is move the blank space up down left or right and it makes sure that its valid(within the gird to do so) 
    which will swap the blank space with the tile corisponding to the action of left right up down 
    
    the transtion takes that action and generates the new state from that action
    
    then we have take that new state and see if weve already seen it as we keep a map of all the states we have seen and if we have seen it we then check if the cost
    to get to this state is less than the one we have and if one of those is true (we havent seen it or it is less than) then we add it to the frontier(list of things to be checked and do the same logic)
    and move on
    
    One thing to note is that this algorithim uses a min heap prioty queue which if you break that down means it uses a heap (tree stucter in a list), and its a min heap
    so its sorted to ahve the smallest thing at the top then priotity is the thing we put in there which means its the value we put in for the cost + huristic
    proity queue means its by prioty and not by time
    
    Some things to note we use nodes for the states to pass around and we use this data structur under to keep track of metrics.
    had to import dataclasses to do so which could be a cool rabit hole to go down to fully understand how these work """

# this is a little object so we dont have to have global varables
# we just make a new one each time and update the things inside then return this object along with the solution
@dataclass
class Metrics:
    nodes_expanded: int = 0
    nodes_generated: int = 0
    max_frontier_size: int = 0
    solution_cost: int = -1
    solution_depth: int = -1
# @dataclass is just a shortcut Python 
# gives you so you don’t have to hand-write boring __init__, __repr__, __eq__, etc., for simple “bags of fields.”

# defintion of Node we will use to pass around
class Node:
    def __init__(self, state, parent=None, action=None, cost=0, depth=0):
        self.state = state  # The current state of the puzzel will be a tuplle
        self.action = action # action made to get here off of the blank spot
        self.parent = parent  # Reference to the node that created this one
        self.cost = cost  # The number of steps to reach this state
        self.depth = depth
# global varables

def actions(state):
    #  we could use yeild here like we learned in operating systems as we are only grabing one thing at a time and we iterate over the list either way so a generator wouldnt be bad
    moves = []
    i = state.index(0)
    r, c = i // 3, i % 3
    #  go through each and see if its valid, if so add it to the list
    if c > 0: moves.append('L')
    if c < 2: moves.append('R')
    if r > 0: moves.append('U')
    if r < 2: moves.append('D')
    # return the list of valid moves on zero space
    return moves

def transition(state,action):
    #  we dont need to check for valid moves as our action function does that
    # get the index where the zero is then get the row and colum
    zero_index = state.index(0)
    r = zero_index // 3
    c = zero_index % 3

    if action == "L":
        # same row but one colum to the left
        swaped_index = r * 3 + (c -1) 
    elif action == "R":
        # same row but one colum to the right
        swaped_index = r * 3 + (c+1)
    elif action == "U":
        # one row up but same colum
        swaped_index = (r-1) * 3 + c
    elif action == "D":
        # one row down but same colum
        swaped_index = (r+1) * 3 + c
    
    # turn tuple into list so we can do swaping like we use to in 2450
    # give them better thing instead of the long name
    i , j = zero_index, swaped_index
    lst = list(state)
    # swap them
    lst[i], lst[j] = lst[j], lst[i]
    # return back the list casted and turned into a tuple
    return tuple(lst)
    


def IsGoal(state,goal):
    return state == goal

def heuristic(state,num,goal):
    # basic
    if num == 0:
        return 0
    
    # how many are out of place
    elif num == 1:
        all_count = 0
        length = len(state)
        # enumerate will give out index, value from what we are going over
        for i,tile in enumerate(state):
            if goal[i] != tile and tile != 0:
                all_count += 1
        return all_count

    # manhantan so the distacne on how much they are out of place
    elif num == 2:

        dist = 0
        for i,tile in enumerate(state):
            if tile == 0:
                continue
            # get the row and collum of the one we have and the goal
            r = i // 3
            c = i % 3
            goal_index = goal.index(tile)
            goal_r = goal_index //3
            goal_c = goal_index % 3
            dist += abs(r - goal_r) + abs(c - goal_c )
        return dist

    else:
        return None

    # logic for all three can split them into there own functions if i need



def f_cost(g_value, state, num, goal):
    return g_value + heuristic(state, num, goal)
# recurse up the parent path
# makes a list and reverses it
# this was copied over as it didnt need a change
def getpath(node):
    path = []
    # Loop from the goal node back to the start
    # apending tuples
    while node:
        path.append((node.state, node.action))
        node = node.parent
    
    # Reverse the path to get it in the correct order (start to goal)
    path = path[::-1]
    
    # Clean up the initial node entry
    if path and path[0][1] is None:
        path[0] = (path[0][0], "Start")
    return path

def A_star_search(start,goal,num):
    #  our min heap queue
    frontier = []
    # will be for our seen state so we can prune things
    # best cost so far for states
    g = {start.state : 0} 
    # generator so we dont compar our states in the min queue
    tie = count()
    heapq.heappush(frontier, (0, next(tie), start))
    metrics = Metrics()
    metrics.max_frontier_size = max(metrics.max_frontier_size , len(frontier))
    while frontier:
        _, _, n = heapq.heappop(frontier)
        # Lazy deletion: if this node's g is not the current best, skip it
        # can use cost becasuse we just add by one dont need another var for g
        if n.cost != g[n.state]:
            continue
        #  if its a goal return the path taken
        if IsGoal(n.state,goal):
            metrics.solution_cost = n.cost
            metrics.solution_depth = n.depth
            return getpath(n) , metrics
        metrics.nodes_expanded += 1
        for a in actions(n.state):
            metrics.nodes_generated += 1
            new_state = transition(n.state,a)
            new_g = n.cost + 1
            if (new_state not in g or  new_g < g[new_state] ):
                # put it in the queue
                new_node = Node(state=new_state, parent=n, action=a,cost=n.cost+1,depth=n.depth + 1)
                g[new_state] = new_g
                # how would i affect one that is already in the heap?
                heapq.heappush(frontier, (f_cost(new_g,new_state,num,goal), next(tie), new_node))
                # check the length
                metrics.max_frontier_size = max(metrics.max_frontier_size , len(frontier))
                


    return None

# this was my test case for the first time
# start = (1,2,3,4,5,0,6,7,8)
# goal = (1,2,3,4,5,6,7,8,0)
# start_node = Node(state=start,parent=None,action=None,cost=0)
# a = A_star_search(start_node,goal,0)

# print(a)
# our cases that we will go over, the data structuer is a list of maps or dictionarys
cases_list = [
    {"start": (0, 8, 3, 2, 1, 7, 6, 5, 4), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (1, 5, 2, 7, 8, 3, 6, 0, 4), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (1, 6, 4, 0, 8, 7, 2, 5, 3), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (1, 7, 3, 5, 0, 8, 2, 6, 4), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (1, 8, 0, 2, 3, 6, 4, 5, 7), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (1, 8, 5, 7, 4, 6, 0, 3, 2), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (2, 5, 3, 1, 8, 0, 4, 6, 7), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (3, 6, 8, 0, 7, 2, 5, 1, 4), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (3, 6, 8, 7, 0, 2, 1, 4, 5), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (3, 7, 1, 5, 6, 8, 0, 4, 2), "goal": (1,2,3,4,5,6,7,8,0)},

    {"start": (4, 7, 8, 5, 0, 3, 6, 1, 2), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (4, 8, 5, 2, 0, 3, 6, 7, 1), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (5, 3, 1, 2, 6, 0, 7, 4, 8), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (5, 8, 7, 1, 4, 0, 6, 3, 2), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (6, 1, 5, 3, 7, 4, 2, 8, 0), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (6, 2, 7, 8, 4, 0, 5, 1, 3), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (6, 5, 2, 8, 7, 0, 4, 3, 1), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (7, 0, 3, 1, 8, 4, 6, 5, 2), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (7, 2, 4, 3, 6, 1, 0, 5, 8), "goal": (1,2,3,4,5,6,7,8,0)},
    {"start": (7, 4, 6, 2, 8, 1, 5, 3, 0), "goal": (1,2,3,4,5,6,7,8,0)},
]


def main():
    print("This is my program for Asigment 2 for AI, the algorthim used for this program was A* and it includes 3 heuristics\nThe program has 20 cases it will run on you will be able to input what case you want to run\nAlso you will be asked for a number that will be between 0 and 2 and it will set which heuristic to use")
    while True:
        try:
            print("Cases")
            for j, i in enumerate(cases_list):
                print("Case "+ str(j) + ": Start: "+ str(i["start"]) + " Goal" + str(i["goal"]))
            x = int(input("Enter what case you want to run: ex for case 19 enter 19  "))
            if(x < 0 or x >= 20):
                print("Please input a valid number")
            else:
                print("enter what heuristic you want\n 0 = zero_heuristic where it returns 0 for the value\n 1 = out of place heuristic where it will count how many are out of place\n 2 = manhantan heuristic where it will calculate how far away the tiles are from there goal")
                while True:
                    try:
                        h = int(input("enter which huristic you want ex...  0  "))
                        if(h <=2 and h >= 0):
                            break
                        else:
                            print("please input a valid number")
                    except ValueError:
                        print("input invalid")
                        pass
                

                start_case = cases_list[x]
                start_node = Node(state=start_case["start"],parent=None,action=None,cost=0)
                output, metrics = A_star_search(start_node,start_case["goal"],h)
                print("\n")
                print("The output of case " + str(x) + ":")
                print(output)
                print("\n")
                print("Metrics")
                print("The amount of nodes expanded: " + str(metrics.nodes_expanded))
                print("The amount of nodes generated: " + str(metrics.nodes_generated) )
                print("The max frontier size: " + str(metrics.max_frontier_size))
                print("The solution cost: " + str(metrics.solution_cost))
                print("The solution depth: " + str(metrics.solution_depth))
                print("\n")

                quit_value = input("Enter q if you want to quit ")
                if quit_value == "q":
                    break
            print("\n")
        except ValueError:
            print("input invalid")
            pass
    print("Good bye...")


if __name__ == '__main__':
    main()


