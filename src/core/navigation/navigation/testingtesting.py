import numpy as np
import time

print(np.arctan2(-0.028,0.544))
# y_start = 0
# y_end = 7
# x_start = 0
# x_end = 7
# grid = np.random.randint(0,10,(10,10))
# print(grid)
# new_grid = grid[y_start:y_end, x_start:x_end]
# print(new_grid)


# for i in range(new_grid.shape[0]):
#     for j in range(new_grid.shape[1]):
#         if new_grid[i,j] == 0:
#             print(f"{i}, {j}")
# n_tries = 10
# idx = np.where(new_grid==0)
# print("idx")
# print(idx)
# start_time = time.time()
# choice_indicies = np.random.randint(0,len(idx[0]))
# print("choice_indicies")
# print(choice_indicies)
# print("valda y värden stor grid")
# print(idx[0][choice_indicies]+y_start)
# print("valda x värden stor grid")
# print(idx[1][choice_indicies]+x_start)
# choice_values = grid[idx[0][choice_indicies]+y_start,idx[1][choice_indicies]+x_start]
# print("choice_values")
# print(choice_values)
# end_time = time.time()
# elapsed_time = end_time-start_time
# print(f"Time taken: {elapsed_time:.4f} seconds")
