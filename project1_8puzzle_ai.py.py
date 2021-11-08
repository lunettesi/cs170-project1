
"""#############################################################
AI Project 1 The Eight Puzzle

Revision History:
v0.1   starting point --- no algorithm yet
v0.2   added PuzzleProblem class,
       added some helper functions 
       added general_search, expand, and uniform_cost_search per handout template
       started to comment out the code from project handout example
v0.3   added A* with the Misplaced Tile Heuristic
       added A* with the Manhattan Distance heuristic 
       added time measurement (print on screen)
v0.4   added load input from json file support with command line input
       enabled support for 8 puzzle, 15 puzzle, 25 puzzle
       argv[1] json file name 
       argv[2] algorithm option
       
       with no command line, default to 8 puzzle

v1.0   added results to both screen and file 
       one sample result 
       ======= Result Begin ===============
       Puzzle state:
       [0, 7, 2]
       [4, 6, 1]
       [3, 5, 8]
       algorithm used: Manhattan Distance Heuristic
       algorithm execution time: 3.2287s
       Total nodes visited was 1758
       Max queue size was 1168
       puzzle solved -- match with goal state
       The depth of the goal node was 24
       =======Result End ===============

not included:
error handling, e.g. assume the entered puzzle is valid
       
References:
1. started with the sample code on project handout, by Dr. Eamonn Keogh
2. used the algorithm pseudo code on project handout, by Dr. Eamonn Keogh
3. the blind search and heuristic search slides from CS170 lecture
4. search internet on some Python language/library/usage details, 
   e.g. heapq, deepcopy of list, etc.
   

"""
import math
import numpy  # for array operation
import heapq  # as priority queue
import copy   # for deep copy
import time   # algorithm execution time
import sys    # for command line inputs
import json   # for json inputs

#helper-functions
def print_puzzle(puzzle):
    for i in range(0, PUZZLE_DIM):
        print(puzzle[i])

def index_2d(myList, v):
    for i, x in enumerate(myList):
        if v in x:
            return (i, x.index(v))

#PuzzleProblem class 
# store puzzle 
#    initial_state, 
#    goal_state,
#    goal_test

class PuzzleProblem:
    GOAL_STATE = []  #added class variable for heuristic calculation
    def __init__(self, initial_state):
        self.initial_state = initial_state  # record initial state
        self.set_goal_state()       # record goal state
        self.visited = []  # keep track visited state
        self.max_queue_size = 0
        PuzzleProblem.GOAL_STATE = self.get_goal_state ()

    def get_initial_state(self):
        return self.initial_state

    def get_goal_state(self):
        return self.goal_state

    def set_goal_state(self):
        lis = []
        for num in range(1, PUZZLE_SIZE + 1):
            lis.append(num)   
        lis.append(0)
        arr = numpy.array(lis)
        self.goal_state = numpy.reshape(arr, (PUZZLE_DIM, PUZZLE_DIM )).tolist()

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

    #find index of SPACE (0)
    row, col = index_2d(state, 0)
    
    if row > 0: # move SPACE up
        state_up = copy.deepcopy(state)
        state_up[row] [col] = state_up[row-1][ col]
        state_up[row-1][col] = 0
        if not problem.is_visited(state_up): 
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1
            heapq.heappush(expanded_nodes, (0, g, 0, state_up))
    
    if row < PUZZLE_DIM -1: # move SPACE down
        state_down = copy.deepcopy(state)
        state_down[row][col] = state_down[row+1][col]
        state_down[row+1][col] = 0
        if not problem.is_visited(state_down):
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1
            heapq.heappush(expanded_nodes, (0, g, 0, state_down))                
        
    if col > 0: # move SPACE left
        state_left = copy.deepcopy(state)
        state_left[row][col] = state_left[row][col-1]
        state_left[row][col-1] = 0
        if not problem.is_visited(state_left):
            # temporary queue tuple(node) (priority, depth, heuristic, state)
            # priority (f), depth(g) , heuristic(h) and state  
            # priority to 0, depth(g) add 1, heuristic to 0
            g = node[1]+1            
            heapq.heappush(expanded_nodes, (0, g, 0, state_left))             
        
    if col < PUZZLE_DIM -1: # move SPACE right
        state_right = copy.deepcopy(state)        
        state_right[row][col] = state_right[row][col+1]
        state_right[row][col+1] = 0
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
        for row in range(0, PUZZLE_DIM):
            for col in range(0, PUZZLE_DIM):
                if state[row][col] > 0 and state[row][col] != PuzzleProblem.GOAL_STATE[row][col]:
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
        for row in range(0, PUZZLE_DIM):
            for col in range(0, PUZZLE_DIM):
                if state[row][col] > 0 and state[row][col] != PuzzleProblem.GOAL_STATE[row][col]:
                    row_goal, col_goal = index_2d(PuzzleProblem.GOAL_STATE, state[row][col])
                    dis = abs(row - row_goal) + abs (col - col_goal)
                    h += dis 
        f = h + g  # use depth(g) + heuristic (g) as priority (f)
        # 
        heapq.heappush(nodes, (f, g, h, state))

def process_cli_input(argv):
    #read-in from command-line, puzzle json file and algorithm to use
    print ("read-in from command line input, puzzle json file and algorithm to use")
    # argv[1] json file format
    # {
    #    "puzzle_type": 8, 
    #    "data": [
    #        [1, 2, 3],
    #        [4, 5, 6],
    #        [7, 8, 0]
    #    ]
    # }        
    f = open (argv[1], "r")
    puzzle_data = json.load(f)
    f.close()       
    puzzle_size = int(puzzle_data['puzzle_type'])
    puzzle_dim = int(math.sqrt(puzzle_size + 1)) # e.g. 3x3 dim for the eight puzzle
    data = puzzle_data['data']
    #argv[2] algorithm 
    # 1 - uniform cost search
    # 2 - Misplaced Tiles Heuristic
    # 3 - Manhattan Distance Heuristic 
    alg = argv[2]    
    return (puzzle_size, puzzle_dim, data, alg)

# Quick testing examples from hand-out
# Below are some built-in puzzles to allow quick testing. 
trivial = [[1, 2, 3],
           [4, 5, 6],
           [7, 8, 0]]
veryEasy = [[1, 2, 3],
            [4, 5, 6],
            [7, 0, 8]]
easy = [[1, 2, 0],
        [4, 5, 3],
        [7, 8, 6]]
doable = [[0, 1, 2],
          [4, 5, 3],
          [7, 8, 6]]
oh_boy = [[8, 7, 1],
          [6, 0, 2],
          [5, 4, 3]]

def init_default_puzzle_mode():
    selected_difficulty = input("You wish to use a default puzzle." 
                                + "Please enter a desired difficulty on a scale from 0 to 4." 
                                + '\n')
    if selected_difficulty == "0":
        print("Difficulty of 'Trivial' selected.")
        return trivial
    elif selected_difficulty == "1":
        print("Difficulty of 'Very Easy' selected.")
        return veryEasy
    elif selected_difficulty == "2":
        print("Difficulty of 'Easy' selected.")
        return easy
    elif selected_difficulty == "3":
        print("Difficulty of 'Doable' selected.")
        return doable
    elif selected_difficulty == "4":
        print("Difficulty of 'Oh Boy' selected.")
        return oh_boy

def process_interactive_input():
    print ("user default -  8 puzzle only support")
    puzzle_size = 8
    puzzle_dim = int(math.sqrt(puzzle_size + 1)) # e.g. 3x3 dim for the eight puzzle        
    puzzle_mode = input("Welcome to an 8-Puzzle Solver." +
                        "Type '1' to use a default puzzle, or '2' to create your own." + '\n')

    if puzzle_mode == "1":
        data = init_default_puzzle_mode()
    if puzzle_mode == "2":
        print("Enter your puzzle, using a zero to represent the blank. " +
          "Please only enter valid 8-puzzles. Enter the puzzle demilimiting " +
          "the numbers with a space. RET only when finished." + '\n')
        puzzle_row_one = input("Enter the first row: ")
        puzzle_row_two = input("Enter the second row: ")
        puzzle_row_three = input("Enter the third row: ")
        puzzle_row_one = puzzle_row_one.split()
        puzzle_row_two = puzzle_row_two.split()
        puzzle_row_three = puzzle_row_three.split()
        for i in range(0, 3):
            puzzle_row_one[i] = int(puzzle_row_one[i])
            puzzle_row_two[i] = int(puzzle_row_two[i])
            puzzle_row_three[i] = int(puzzle_row_three[i])

        data = [puzzle_row_one, puzzle_row_two, puzzle_row_three]
    alg = input("Select algorithm. \n" +
                    "(1) for Uniform Cost Search,\n" +
                    "(2) for the Misplaced Tile Heuristic, \n" +
                    "or (3) the Manhattan Distance Heuristic.\n" ) 
    return (puzzle_size, puzzle_dim, data, alg)

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
        for i in range(0, PUZZLE_DIM):
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
        PUZZLE_SIZE, PUZZLE_DIM, user_puzzle, algorithm = process_cli_input(sys.argv)
    # use default - interactive input
    else: 
        PUZZLE_SIZE, PUZZLE_DIM, user_puzzle, algorithm = process_interactive_input()
    
    print_puzzle(user_puzzle)
    problem = PuzzleProblem(user_puzzle)
    
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

       
   
