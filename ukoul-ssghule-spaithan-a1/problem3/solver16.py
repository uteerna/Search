#!/usr/bin/env python
"""
########################################################################################################################
# 15 puzzle problem is a search problem in which we move the blank tile on the board so that we reach the goal state   #
#                                                                                                                      #
# Initial State: Any random arrangement of the board with 15 numbered tiles and a blank tile which is represented by 0 #
#                                                                                                                      #
# Goal State: Arrangement of the board in such a way that all numbers are in ascending order with the blank tile at the#
# last position of the board                                                                                           #
#                                                                                                                      #
# State Space: Any possible arrangement of the board                                                                   #
#                                                                                                                      #
# Successor Function: The successor function will give us a list of successors by generating the next state by moving  #
# the blank tile in either left, right, up or down positions. For each of these successors we will further have a list #
# of successors wherein we cannot move in the opposite direction of the previous move. Example, if my blank tile is    #
# moved left then i cannot have an immediate right movement.                                                           #
#                                                                                                                      #
# Heuristic value: The heuristic value used here is Manhattan Distance wherein we calculate the distance of numbered   #
# tiles from their original position. The tiles distance is divided by 3 as we can consider 1, 2 or 3 moves in the same#
# direction as one move. Manhattan distance divided by 3 is admissible as it would not overestimate the heuristic      #
# function. After the manhattan distance, the check for linear conflict is done and if there is a conflict then the    #
# heuristic value is incremented by 2 for those tiles. This method provides us with an admissible heuristic.           #
#                                                                                                                      #
# Cost: For each move there is no cost associated with the transitions.                                                #
#                                                                                                                      #
# Algorithm:                                                                                                           #
# 1. Read the initial board from the file.                                                                             #
# 2. Check if the board is solvable. If it is, then proceed to the next step else exit.                                #
# 3. Set the move count and the heuristic value                                                                        #
# 4. Check if the state exists in fringe, if it does then repeat the following steps:                                  #
# 4a. Check if the state is visited then append it to the visited list.                                                #
# 4b. Check if the goal state is reached, if it is then exit else continue                                             #
# 4c. If the successor state is in visited then do not calculate a heuristic value for it, else calculate a heuristic  #
#                                                                                                                      #
# All state objects in the fringe. State objects contains the board, steps list, heuristic value, row, column value    #
# and the move count for moving any tile on the board.                                                                 #
# In this search problem i have used Search algorithm 2                                                                #
########################################################################################################################
"""

import sys
from copy import deepcopy
import Queue
import math
import time

# 15 puzzle solver. A* algorithm used
initial_board = [[0] * 4] * 4

# Goal State
goal_state = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 0]]


# Class for state with heuristic value
class State(object):
    board = [[]]
    steps_list = []
    heuristic_val = 0
    # evaluation = None
    row = 0
    column = 0
    move_count = 0

    def __init__(self, board, steps_list, row, col):
        self.board = board
        self.steps_list = steps_list
        # self.evaluation = len(steps_list) + self.heuristic_val
        self.row = row
        self.column = col

    def setmovecount(self, move_count):
        self.move_count = move_count

    def set_heuristic(self, h):
        self.heuristic_val = h


# Function to read initial board file
def read_initial_board(filename):
    f = open(filename, "r")
    for i in range(4):
        input_row = f.readline().strip().split(" ")
        initial_board[i] = [int(j) for j in input_row]
    return initial_board


# Function to print the board
def print_board(board):
    for i in range(4):
        for j in range(4):
            print board[i][j],
            print " ",
        print
    return


# Check Goal State
def is_goal(s):
    if goal_state == s.board:
        return True
    else:
        return False


# Get row of blank tile which is represented by 0
def get_row_tile(tile, board):
    for i in range(0, 4):
        for j in range(0, 4):
            if board[i][j] == tile:
                return i
    return -1


# Get column of blank tile which is represented by 0
def get_column_tile(tile, board):
    for i in range(0, 4):
        for j in range(0, 4):
            if board[i][j] == tile:
                return j
    return -1


# Swapping a blank tile with a numbered tile
def swap_tile(board, row1, column1, row2, column2, x):
    for i in range(x):
        if row1 == row2:
            if column2 > column1:
                board[row1][column1 + i] = board[row2][column1 + i + 1]
            if column1 > column2:
                board[row1][column1 - i] = board[row2][column1 - i - 1]
        if column1 == column2:
            if row2 < row1:
                board[row1 - i][column1] = board[row1 - i - 1][column2]
            if row1 < row2:
                board[row1 + i][column2] = board[row1 + i + 1][column2]
    board[row2][column2] = 0

    return board


# Successor function to generate next states
def successor(state, row, column):
    succ_states = []
    steps_list = deepcopy(state.steps_list)
    x = 0
    for i in range(row - 1, -1, -1):
        x += 1
        result = swap_tile(deepcopy(state.board), row, column, row - x, column, x)
        steps_list.append(str('D' + str(x) + str(column + 1)))
        state1 = State(result, steps_list, (row - x), column)
        succ_states.append(state1)
        steps_list = steps_list[:-1]

    x = 0
    for i in range(row + 1, 4):
        x += 1
        result = swap_tile(deepcopy(state.board), row, column, row + x, column, x)
        steps_list.append(str('U' + str(x) + str(column + 1)))
        state1 = State(result, steps_list, (row + x), column)
        succ_states.append(state1)
        steps_list = steps_list[:-1]

    x = 0
    for i in range(column + 1, 4):
        x += 1
        result = swap_tile(deepcopy(state.board), row, column, row, column + x, x)
        steps_list.append(str('L' + str(x) + str(row + 1)))
        state1 = State(result, steps_list, row, column + x)
        succ_states.append(state1)
        steps_list = steps_list[:-1]

    x = 0
    for i in range(column - 1, -1, -1):
        x += 1
        result = swap_tile(deepcopy(state.board), row, column, row, column - x, x)
        steps_list.append(str('R' + str(x) + str(row + 1)))
        state1 = State(result, steps_list, row, column - x)
        succ_states.append(state1)
        steps_list = steps_list[:-1]

    return succ_states


# Function to display result
def display(steps_list):
    for i in steps_list:
        print i + " ",
    return


# Calculate heuristic value of state.
def cal_heuristic(board):
    row = [4, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3]
    col = [4, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3]
    heuristic_value = 0
    for i in range(4):
        for j in range(4):
            if board[i][j] != 0:
                heuristic_value += math.ceil(math.fabs(row[board[i][j]] - i) / 3.0) + math.ceil(
                    math.fabs(col[board[i][j]] - j) / 3.0)
            if row[board[i][j]] == i:
                for k in range(j+1, 4):
                    if row[board[i][k]] == i:
                        if col[board[i][k]] < col[board[i][j]]:
                            heuristic_value += 2
            if col[board[i][j]] == j:
                for k in range(i + 1, 4):
                    if col[board[k][j]] == j:
                        if row[board[k][j]] < row[board[i][j]]:
                            heuristic_value += 2

    return heuristic_value


# Checking if the board is solvable
def check_board(board):
    inversion = 0
    rows = [0] * 16
    for i in range(4):
        for j in range(4):
            rows[4 * i + j] = board[i][j]
    for i in range(16):
        if rows[i] != 0 or rows[i] != 1:
            for j in range(i + 1, 16):
                if rows[i] > rows[j] and rows[j]:
                    inversion += 1
    blank = 4 - get_row_tile(0, board)
    if blank & 1:
        # print "false"
        return not (inversion & 1)
    else:
        # print "true"
        return inversion & 1


# Solve function for 15 puzzle problem
def solve(board):
    a = time.time()
    fringe = Queue.PriorityQueue()

    row = get_row_tile(0, board)
    column = get_column_tile(0, board)
    state = State(board, [], row, column)
    state.setmovecount(0)
    state.set_heuristic(cal_heuristic(state.board))
    fringe.put((0, state))
    visited = []
    openi = [state.board]
    while not fringe.empty():
        s = fringe.get()
        s = s[1]

        # print "test", s.board, s.steps_list, s.row, s.column, s.move_count
        visited.append(s)
        if is_goal(s):
            # print s.move_count
            print " ".join(s.steps_list)
            b = time.time()
            print "Total time taken: ", (b-a)
            exit()
        current_successors = successor(s, s.row, s.column)
        for i in current_successors:
            # print "it", i.board, i.steps_list, i.heuristic_val, i.move_count + i.heuristic_val, i.move_count
            i.setmovecount(s.move_count + 1)
            i.set_heuristic(cal_heuristic(i.board))
            flag = 0

            for x in visited:
                if i.board == x.board:
                    flag = 1
            for y in openi:
                if i.board == y:
                    flag = 1
            if flag == 0:
                if i.heuristic_val != 0:
                    fringe.put((i.move_count + i.heuristic_val, i))
                    openi.append(i.board)
                    # print "i", i.board, i.steps_list, i.heuristic_val, i.move_count + i.heuristic_val, i.move_count
                else:
                    fringe.put((i.heuristic_val, i))

        # print "hello"
    # print board
    return


# Read input filename
filename = sys.argv[1]

# Read the input file
initial_board = read_initial_board(filename)
#initial_board = read_initial_board("C:\\Python Codes\\Artificial Intelligence\\test.txt")

# Print initial board
print "Initial Board:", print_board(initial_board)

# Checking if the board is solvable, if it is then looking for a solution else exiting the code
if check_board(initial_board):
    print "Solving.."
    solve(initial_board)
else:
    print "The board is not solvable!!!!"
