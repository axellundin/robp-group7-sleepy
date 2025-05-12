import numpy as np

def dist(x_1, y_1, x_2, y_2):
    return (np.sqrt((x_1-x_2)**2+(y_1-y_2)**2))


print("exploration 2")

print("sphere 1")
print(dist(83, 172	,83,	175	)) # sphere 1
print("toy 1")
# print(dist(292, 74	,296,	75	)) # toy 1
print("undetected")
print("box 1")
# print(dist(286, 193	,325,	231	)) # box 1
print("undetected")
print("sphere 2")
print(dist(439, 196	,424,	187		)) # sphere 2
print("cube 1")
print(dist(456, -30	,467,	-40			)) # cube 1
print("--------------------------------------------------------------------------------")
# # exploration 1
# print("exploration 1")

# print("cube 1")
# print(dist(87, -34	,86,	-35		)) # cube 1

# print("sphere 1")
# print(dist(269, 95	,278,	93	)) # sphere 1

# print("box")
# print(dist(522, 206	,533,	210		)) # box 1

# print("cube 2")
# print(dist(456, -30	,467,	-28		)) # cube 2

# print("toy")
# print(dist(599, 52	,607	,42	)) # toy