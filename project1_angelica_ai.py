
"""
#############################################################
AI Project 1 - THE ANGELICA PUZZLE

Puzzle Problem Description:
    http://jnsilva.ludicum.org/HMR13_14/536.pdf  page 149

Revision History:
v1.0   1. Started from 8 puzzle code v1.0 (file name is project1_ai.py)
               1.5. All of my iterations for the 8-puzzle are included 
                    in the header of the project1_ai.py file. 
       2. Removed support of interactive inputs, leave only support 
          of reading puzzle input from .json file (interactive inputs are
          still on the 8-puzzle solver)
       3. Updated input json format to include row, col, and goal.
          example:
       {
           "row": 3,
           "col": 3,    
           "data": [
           ["A", "N", "G"],
           ["E", "L", "I"],
           [" ", "C", "A"]
           ],
           "goal": [
           ["A", "N", "G"],
           ["E", "L", "I"],
           ["C", "A", " "]
           ]
      }
      4. Updated mahattan distance heuristic to distinguish the two "A"
         separately, each based off their own goal row/col to calculate the 
         h value.
      5. No changes required for most of the functions and algorithms. Expand
         states based off SPACE (' ').
   
############################################################
"""
import math
#import numpy  # for array operation
import heapq  # as priority queue
import copy   # for deep copy
import time   # algorithm execution time
import sys    # for command line inputs
import json   # for json inputs

#helper-functions
def print_puzzle(puzzle):
    for i in range(0, PUZZLE_ROW):
        print(puzzle[i])

# there might be repeat characters
def index_2d(myList, v):
    rowL = []
    colL = []
    for i, x in enumerate(myList):
        if v in x:
            rowL.append(i)
            colL.append(x.index(v))
    return (rowL, colL)

#PuzzleProblem class 
# store puzzle 
#    initial_state, 
#    goal_state,
#    goal_test

class PuzzleProblem:
    GOAL_STATE = []  #added class variable for heuristic calculation
    def __init__(self, initial_state, goal_state):
        self.initial_state = initial_state  # record initial state
        self.set_goal_state(goal_state)       # record goal state
        self.visited = []  # keep track visited state
        self.max_queue_size = 0
        PuzzleProblem.GOAL_STATE = self.get_goal_state ()

    def get_initial_state(self):
        return self.initial_state

    def get_goal_state(self):
        return self.goal_state

    def set_goal_state(self, goal_state):
        self.goal_state = copy.deepcopy(goal_state)


    def set_max_queue_size(self, queue_size):
        if (queue_size > self.max_queue_size):
            self.max_queue_size = queue_size
    
    def get_max_queue_size(self):
        return self.max_queue_size     
            
    def add_visited(self, state):
        self.visited.append(state)

    def is_visited(self, state):
        for visited_state in self.visited:
            if (state == visited_state):
                return True
        return False
    
    def get_total_visited(self):
        return len(self.visited)
    
    def goal_test(self, state):
        return (state == self.goal_state)
    

    def print_problem(self):
        print("Initial State")
        print_puzzle(self.initial_state)
        print("Goal State")
        print_puzzle(self.goal_state)  

        
"""
function general-search(problem, QUEUEING-FUNCTION) 
    nodes = MAKE-QUEUE(MAKE-NODE(problem.INITIAL-STATE)) 
    loop do
        if EMPTY(nodes) then return "failure" 
        node = REMOVE-FRONT(nodes) 
        if problem.GOAL-TEST(node.STATE) succeeds then return node
        nodes = QUEUEING-FUNCTION(nodes, EXPAND(node, problem.OPERATORS)) 
    end
"""

def general_search(problem, queueing_function):
    found = False
    depth = 0
    nodes = []
    # queue tuple(node) (priority, depth, heuristic, state)
    # added heuristic for A* algorithms
    heapq.heappush(nodes, (0, 0, 0, problem.get_initial_state()))
    while len(nodes):
        problem.set_max_queue_size(len(nodes))  # keep track of largest queue size for metrics reporting
        node = heapq.heappop(nodes)
        state = node[len(node)-1]
        print_puzzle(state)
        if not problem.is_visited(state):
            problem.add_visited(state)
        if problem.goal_test(state): 
            #found goal node
            return (node)  # return node found
        print ("call queueing_function to expand searching nodes\n")
        queueing_function(nodes, expand(node, problem))
        depth += 1
    return None  # return failure and empty state

#expand to adjacent states - left, right, up, down
def expand(node, problem):
    expanded_nodes = []
    state = node[len(node)-1]

    #find index of SPACE (' ')
    rowL, colL = index_2d(state, ' ')
    
    row = rowL[0]
    col = colL[0]
    if row > 0: # move SPACE up
        state_up = copy.deepcopy(state)
        state_up[row] [col] = state_up[row-1][ col]
        state_up[row-1][col] = ' '
        if not problem.is_visited(state_up): 
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1
            heapq.heappush(expanded_nodes, (0, g, 0, state_up))
    
    if row < PUZZLE_ROW -1: # move SPACE down
        state_down = copy.deepcopy(state)
        state_down[row][col] = state_down[row+1][col]
        state_down[row+1][col] = ' '
        if not problem.is_visited(state_down):
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1
            heapq.heappush(expanded_nodes, (0, g, 0, state_down))                
        
    if col > 0: # move SPACE left
        state_left = copy.deepcopy(state)
        state_left[row][col] = state_left[row][col-1]
        state_left[row][col-1] = ' '
        if not problem.is_visited(state_left):
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1            
            heapq.heappush(expanded_nodes, (0, g, 0, state_left))             
        
    if col < PUZZLE_COL -1: # move SPACE right
        state_right = copy.deepcopy(state)        
        state_right[row][col] = state_right[row][col+1]
        state_right[row][col+1] = ' '
        if not problem.is_visited(state_right):
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1            
            heapq.heappush(expanded_nodes, (0, g, 0, state_right)) 
    return expanded_nodes

def uniform_cost_search(nodes, expanded_nodes):
    while len(expanded_nodes):
        # queue tuple(node) (priority, depth, heuristic, state) to search nodes
        # priority (f), depth(g) , heuristic(h) and state     
        f, g, h, state = heapq.heappop(expanded_nodes)
        f = g # use depth(g) as priority (f)
        heapq.heappush(nodes, (f, g, h, state))

def misplaced_tile_heuristic_search(nodes, expanded_nodes):
    while len(expanded_nodes):
        # queue tuple(node) (priority, depth, heuristic, state) to search nodes
        # priority (f), depth(g) , heuristic(h) and state     
        f, g, h, state = heapq.heappop(expanded_nodes)
        # calculate h - misplaced tiles
        h = 0
        for row in range(0, PUZZLE_ROW):
            for col in range(0, PUZZLE_COL):
                if state[row][col] != ' ' and state[row][col] != PuzzleProblem.GOAL_STATE[row][col]:
                    h += 1
        f = h + g  # use depth(g) + heuristic (g) as priority (f)
        # 
        heapq.heappush(nodes, (f, g, h, state))

def manhattan_distance_heuristic_search(nodes, expanded_nodes):
    while len(expanded_nodes):
        # queue tuple(node) (priority, depth, heuristic, state) to search nodes
        # priority (f), depth(g) , heuristic(h) and state     
        f, g, h, state = heapq.heappop(expanded_nodes)
        # calculate h - manhattan distance 
        h = 0
        aflag = 0 
        for row in range(0, PUZZLE_ROW):
            for col in range(0, PUZZLE_COL):
                if state[row][col] != ' ' and state[row][col] != PuzzleProblem.GOAL_STATE[row][col]:
                    row_goal, col_goal = index_2d(PuzzleProblem.GOAL_STATE, state[row][col])
                    if state[row][col] == 'A':
                        if aflag == 0: # first 'A'
                            dis = abs(row - row_goal[0]) + abs(col - col_goal[0])
                            aflag +=1
                        else: # second 'A'
                            dis = abs(row - row_goal[1]) + abs(col - col_goal[1])
                    else: # other letters
                        dis = abs(row - row_goal[0]) + abs(col - col_goal[0])    
                    h += dis 
        f = h + g  # use depth(g) + heuristic (g) as priority (f)
        # 
        heapq.heappush(nodes, (f, g, h, state))

def process_cli_input(argv):
    #read-in from command-line, puzzle json file and algorithm to use
    print ("read-in from command line input, puzzle json file and algorithm to use")
    # argv[1] json file format
    # {
    #"row": 3,
    #"col": 3,    
    #"data": [
    #    ["A", "N", "G"],
    #    ["E", "L", "I"],
    #    [" ", "C", "A"]
    #],
    #"goal": [
    #    ["A", "N", "G"],
    #    ["E", "L", "I"],
    #    ["C", "A", " "]
    #]
    # }        
    f = open (argv[1], "r")
    puzzle_data = json.load(f)
    f.close()       
    puzzle_row = int(puzzle_data['row'])
    puzzle_col = int(puzzle_data['col'])
    data = puzzle_data['data']
    goal = puzzle_data['goal']
    #argv[2] algorithm 
    # 1 - uniform cost search
    # 2 - Misplaced Tiles Heuristic
    # 3 - Manhattan Distance Heuristic 
    alg = argv[2]    
    return (puzzle_row, puzzle_col, data, goal, alg)


def report_to_screen(problem, result_node, elapsed_time, algorithm):
    # reporting on screen
    print("\n======= Result Begin ===============")
    print("Puzzle state:")
    print_puzzle(problem.get_initial_state())
    if(algorithm == "1"):
        print("algorithm used: uniform cost search")
    elif(algorithm == "2"):
        print("algorithm used: Misplaced Tile Heuristic")
    elif(algorithm == "3"):
        print("algorithm used: Manhattan Distance Heuristic") 
        
    print ("algorithm execution time: " + str(round(elapsed_time, 4)) + "s")
    print ("Total nodes visited was %d" % problem.get_total_visited())    
    print ("Max queue size was %d" % problem.get_max_queue_size())   
    if not result_node:
        print ("failed search")
    else:
        print ("puzzle solved -- match with goal state")
        f, g, h, state = result_node   
        print ("The depth of the goal node was %d" % g)
    
    print("======= Result End ===============\n\n")

RESULT_FILE = True

def report_to_file(problem, result_node, elapsed_time, algorithm):
    if RESULT_FILE:
        # report to file

        resultstr = [] 
        resultstr.append(str("\n======= Result Begin ===============\n"))
        resultstr.append(str("Puzzle state:\n"))
        state = problem.get_initial_state()
        for i in range(0, PUZZLE_ROW):
            resultstr.append(str(state[i]) + "\n")        
        if(algorithm == "1"):
            resultstr.append("algorithm used: uniform cost search\n")
        elif(algorithm == "2"):
            resultstr.append("algorithm used: Misplaced Tile Heuristic\n")
        elif(algorithm == "3"):
            resultstr.append("algorithm used: Manhattan Distance Heuristic\n") 
        
        resultstr.append("algorithm execution time: " + str(round(elapsed_time, 4)) + "s" + "\n")
        resultstr.append("Total state visited was %d" % problem.get_total_visited() + "\n") 
        resultstr.append("Max queue size was %d" % problem.get_max_queue_size() + "\n")   
        if not result_node:
            resultstr.append("failed search\n")
        else:
            resultstr.append("puzzle solved -- match with goal state\n")
            f, g, h, state = result_node   
            resultstr.append("The depth of the goal node was %d" % g + "\n")
    
        resultstr.append("=======Result End ===============\n\n")

        # reporting into result file
        resfile = open('result.txt', 'a')
        resfile.writelines(resultstr)
        resfile.close()    

# main driver
if __name__ == "__main__":
    # use command line input json file and algorithm type
    # e.g. python project1_ai.py 8puzzle_depth12.json 1
    if (len(sys.argv) > 1 and len(sys.argv) == 3): 
        PUZZLE_ROW, PUZZLE_COL, user_puzzle, PUZZLE_GOAL, algorithm = process_cli_input(sys.argv)
    else: 
        print("Error: input not matched expected")
        print("e.g. python project1_angelica_ai.py angelica_1.json")
        exit(1)        
    
    print_puzzle(user_puzzle)
        
    problem = PuzzleProblem(user_puzzle, PUZZLE_GOAL)
    
    start_time = time.time()

    # call search algorithm
    if algorithm == "1":
        print ("starting search with (1) Uniform Cost Search")
        result_node = general_search(problem, uniform_cost_search)
    elif algorithm == "2":
        print ("starting search with (2) Misplaced Tile Heuristic")
        result_node = general_search(problem, misplaced_tile_heuristic_search)
    elif algorithm == "3":
        print ("starting search with (3) Manhattan Distance Heuristic")
        result_node = general_search(problem, manhattan_distance_heuristic_search)
    
    end_time = time.time()  
    elapsed_time = end_time - start_time;  # execution time
     
    #reporting results     
    report_to_screen(problem, result_node, elapsed_time, algorithm)
    report_to_file(problem, result_node, elapsed_time, algorithm)


       
   
