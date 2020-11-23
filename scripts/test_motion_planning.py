from motion_planning import get_path_from_A_star

if __name__ == '__main__':
    start = (0, 0) # this is a tuple data structure in Python initialized with 2 integers
    goal = (-5, -2)
    obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
    path = get_path_from_A_star(start, goal, obstacles)
    print(path)
