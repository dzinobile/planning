import numpy as np
import random
import pygame
import time

#Generate random initial state
init_state = list(range(0,9))

#Function to check if random initial state is solvable
def has_solution(s):
    sr= [s[0],s[3],s[6],s[1],s[4],s[7],s[2],s[5],s[8]] #reorder list for ease of programming
    inv_count = 0 #number of inversions
    for i in range(0,9):
        for j in range(i+1, 9):
            if sr[i] != 0 and sr[j] != 0 and sr[i] > sr[j]: #count number of inversions
                inv_count += 1 
    if inv_count%2 == 0: #Check if number of inversions is even or odd
        return True
    else:
        return False

#Randomize and check initial states until a solvable one is found
solvable = False
while not solvable:
    random.shuffle(init_state)
    solvable = has_solution(init_state)


#Uncomment line below for hardest starting conditions
#init_state = [8, 2, 3, 6, 5, 0, 7, 4, 1]

#Define goal state
goal = [1, 4, 7, 2, 5, 8, 3, 6, 0]

#List of the coordinates corresponding to position in state lists
location_list = [(0,0),(1,0),(2,0),
                 (0,1),(1,1),(2,1),
                 (0,2),(1,2),(2,2)]

#Function for locating the blank space coordinates
def locate_blank(state):
    index = state.index(0)
    return location_list[index]

#Function moves blank space up if valid move
def move_up(state):
    
    temp_state = state.copy()
    blank_pos = locate_blank(temp_state)
    blank_index = location_list.index(blank_pos)
    
    if blank_pos[0] == 0:
        return False, temp_state
    else:
        above_pos = (blank_pos[0]-1,blank_pos[1])
        above_index = location_list.index(above_pos)
        above_value = temp_state[above_index]
        temp_state[blank_index] = above_value
        temp_state[above_index] = 0
        return True, temp_state
    
#Function moves blank space down if valid move
def move_down(state):
    
    temp_state = state.copy()
    blank_pos = locate_blank(temp_state)
    blank_index = location_list.index(blank_pos)
    
    if blank_pos[0] == 2:
        return False, temp_state
    else:
        below_pos = (blank_pos[0]+1,blank_pos[1])
        below_index = location_list.index(below_pos)
        below_value = temp_state[below_index]
        temp_state[blank_index] = below_value
        temp_state[below_index] = 0
        return True, temp_state
    
#Function moves blank space left if valid move
def move_left(state):
    
    temp_state = state.copy()
    blank_pos = locate_blank(temp_state)
    blank_index = location_list.index(blank_pos)
    
    if blank_pos[1] == 0:
        return False, temp_state
    else:
        left_pos = (blank_pos[0],blank_pos[1]-1)
        left_index = location_list.index(left_pos)
        left_value = temp_state[left_index]
        temp_state[blank_index] = left_value
        temp_state[left_index] = 0
        return True, temp_state

#Function moves blank space right if valid move
def move_right(state):
    
    temp_state = state.copy()
    blank_pos = locate_blank(temp_state)
    blank_index = location_list.index(blank_pos)
    
    if blank_pos[1] == 2:
        return False, temp_state
    else:
        right_pos = (blank_pos[0], blank_pos[1]+1)
        right_index = location_list.index(right_pos)
        right_value = temp_state[right_index]
        temp_state[blank_index] = right_value
        temp_state[right_index] = 0
        return True, temp_state

#Initialize list with node index, node parent index, and node state
nodes_info = [(0, 0, init_state)]

#Initialize set of checked nodes (set for lookup optimization)
nodes_set = set(tuple(init_state)) 


i = 1 #next node index will be 1
p = 0 #next node parent is initial state, so node parent index set to 0


#BFS search until target state found
goal_found = False
while not goal_found: #search until goal found
    parent = nodes_info[p][2] #search from next parent node
    for move_func in [move_up, move_down, move_left, move_right]:
        v,s = move_func(parent)
        if v: #check if move is valid
            if tuple(s) not in nodes_set: #check if new state already checked
                nodes_set.add(tuple(s))
                nodes_info.append((i, p, s))
                if s == goal:
                    goal_found = True
                    break
                i += 1 #increment node index
    p += 1 #increment parent node index

#Create list of just the nodes
nodes = []
for line in nodes_info:
    nodes.append(line[2])

#Find node path
node_path = []
node_path.append(goal) #Add goal as first item
goal_index = nodes.index(goal) #Find node index of the goal
p_index = nodes_info[goal_index][1] #Find parent index of goal
while p_index != 0: #Find parent index of each parent index until reaching 0
    parent = nodes_info[p_index]
    node_path.append(parent[2])
    p_index = parent[1]
node_path.append(init_state) #Add initial state as last item
node_path.reverse() #Reverse order

#Write node path to file
with open("nodePath.txt", "w") as file:
    for item in node_path:
        file.write(" ".join(map(str, item)) + "\n")

#Write nodes to file
with open("Nodes.txt","w") as file:
    for item in nodes:
        file.write(" ".join(map(str, item)) + "\n")

#Initialize node info file with header    
with open("NodesInfo.txt","w") as node:
    node.write(f"{'Node_index ':<10}{' Parent_Node_Index ':<18}{' Node '}\n")

#Write node info to file   
with open("NodesInfo.txt","w") as file:
    for item in nodes_info:
        node_index, parent_index, state = item
        state_str = " ".join(map(str, state))
        file.write(f"{node_index:<12}{parent_index:<19}{state_str}\n")
        

# open the file for reading
with open('nodePath.txt', 'r') as f:
    # read the content of the file
    content = f.read()

# split the content by blank line to get a list of matrix strings
strings = content.strip().split('\n')

# create a list of 3x3 matrices from the matrix strings
track = []
for string in strings:
    values = list(map(int, string.split()))
    state = np.array(values).reshape(3, 3)
#     print(state)
    track.append(state.T)
# track.reverse()
# initialize pygame
pygame.init()

# set the size of the game window
window_size = (300, 300)
screen = pygame.display.set_mode(window_size)

# set the title of the game window
pygame.display.set_caption("8 Puzzle Game")

# define colors
white = (255, 255, 255)
black = (0, 0, 0)
grey = (128, 128, 128)

# set the font
font = pygame.font.SysFont('Arial', 30)

# define the size of the puzzle grid
grid_size = 3
cell_size = 100

# define the position of the puzzle numbers
puzzle_positions = [(i % grid_size, i // grid_size) for i in range(grid_size ** 2)]

# define the position of the empty cell
empty_position = puzzle_positions[-1]



# define a function to draw the puzzle board
def draw_board():
    for i in range(grid_size):
        for j in range(grid_size):
            x = j * cell_size
            y = i * cell_size
            index = puzzle_positions.index((j, i))
            number = puzzle_numbers[index]
            if number != 0:
                pygame.draw.rect(screen, white, (x, y, cell_size, cell_size))
                text = font.render(str(number), True, black)
                text_rect = text.get_rect(center=(x+cell_size/2, y+cell_size/2))
                screen.blit(text, text_rect)
            else:
                pygame.draw.rect(screen, grey, (x, y, cell_size, cell_size))

# define the main game loop
running = True

i=0
while running  :
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # clear the screen
    screen.fill(grey)
    
#     update position
    if i<len(track):
        puzzle_numbers=track[i].flatten()
        i+=1
    # draw the puzzle board
    draw_board()
    time.sleep(1)
    # update the screen
    pygame.display.update()
    

# quit pygame
pygame.quit()
