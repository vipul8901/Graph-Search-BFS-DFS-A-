# Graph-Search-BFS-DFS-A-

Some search solutions for 8 puzzle game, implementing these algorithms:

Breadth-First Search
Depth-First Search
A*
1. 8 Puzzle game
8puzzle

The 8-puzzle (also called Gem Puzzle, Boss Puzzle, Game of Fifteen, Mystic Square and many others) is a 3x3 sliding puzzle that consists of a frame of eight numbered square tiles in random order with one tile missing

2. Conceptual background algorithm
Breadth-First Search
BFS

Is an algorithm for traversing or searching tree or graph data structures. It starts at the tree root (or some arbitrary node of a graph, sometimes referred to as a 'search key), and explores all of the neighbor nodes at the present depth prior to moving on to the nodes at the next depth level.

It uses the opposite strategy as depth-first search, which instead explores the highest-depth nodes first before being forced to backtrack and expand shallower nodes.

BFS and its application in finding connected components of graphs were invented in 1945 by Konrad Zuse and Michael Burke, in their (rejected) Ph.D. thesis on the Plankalkül programming language, but this was not published until 1972. It was reinvented in 1959 by Edward F. Moore, who used it to find the shortest path out of a maze, and later developed by C. Y. Lee into a wire routing algorithm (published 1961).

Link: wikipedia -> https://en.wikipedia.org/wiki/Breadth-first_search

Depth-First Search
DFS

Is an algorithm for traversing or searching tree or graph data structures. The algorithm starts at the root node (selecting some arbitrary node as the root node in the case of a graph) and explores as far as possible along each branch before backtracking.

A version of depth-first search was investigated in the 19th century by French mathematician Charles Pierre Trémaux as a strategy for solving mazes

Link: wikipedia -> https://en.wikipedia.org/wiki/Depth-first_search

A*
Astar

A* (pronounced as "A star") is a computer algorithm that is widely used in pathfinding and graph traversal, which is the process of plotting an efficiently directed path between multiple points, called "nodes". It enjoys widespread use due to its performance and accuracy. However, in practical travel-routing systems, it is generally outperformed by algorithms which can pre-process the graph to attain better performance, although other work has found A* to be superior to other approaches.

Peter Hart, Nils Nilsson and Bertram Raphael of Stanford Research Institute (now SRI International) first published the algorithm in 1968.[3] It is an extension of Edsger Dijkstra's 1959 algorithm. A* achieves better performance by using heuristics to guide its search.

Link: wikipedia -> https://en.wikipedia.org/wiki/A*_search_algorithm

3. Heuristic selected
In the A* solution the manhattan distance is implemented as a function of estimating the distance to goal, by assigning negative score in each number for the total number of squares away from its goal possition

values_0 = [0,1,2,1,2,3,2,3,4]
values_1 = [1,0,1,2,1,2,3,2,3]
values_2 = [2,1,0,3,2,1,4,3,2]
values_3 = [1,2,3,0,1,2,1,2,3]
values_4 = [2,1,2,1,0,1,2,1,2]
values_5 = [3,2,1,2,1,0,3,2,1]
values_6 = [2,3,4,1,2,3,0,1,2]
values_7 = [3,2,3,2,1,2,1,0,1]
values_8 = [4,3,2,3,2,1,2,1,0]

def calculate_manhattan_dist(state):

    """calculate the manhattan distance of a tile"""

    manh_d=sum( abs(x//3-i//3)+abs(x%3-i%3) for i,x in enumerate(state.config) if x!=0)
        
    return manh_d
    
4. Project input
Execute python via comand prompt:

python driver.py 'method' 'board state'
Where method is the algorithm selected for finding a solution path, and board state is the initial board of the game to start finding a solvable path

Example bfs:

python driver.py bfs 8,6,4,2,1,3,5,7,0
Example dfs:

python driver.py dfs 1,2,5,3,4,8,6,7,0
Example ast:

python driver.py ast 1,2,5,3,4,8,6,7,0
5. Project output

Generate a plain text file that ilustrate the variables of the path that best solve the starting board, Example:

path_to_goal: ['Left', 'Up', 'Up', 'Left', 'Down', 'Right', 'Down', 'Left', 'Up', 'Right', 'Right', 'Up', 'Left', 'Left', 'Down', 'Right', 'Right', 'Up', 'Left', 'Down', 'Down', 'Right', 'Up', 'Left', 'Up', 'Left']
cost_of_path: 26
nodes_expanded: 1001
search_depth: 26
max_search_depth: 26
running_time: 0.09574413
max_ram_usage: 15.38671875
