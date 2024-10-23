
"""
 Enter 1 or 2 in command line to visualize path in path.txt in respective environment.
"""
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import sys

data = numpy.loadtxt('path.txt')
fig, ax = plt.subplots()
ax.plot(data[:, 0],data[:, 1],'.-')

ax.set_xlim(0, 9)  
ax.set_ylim(0, 9)
ax.set_aspect('equal') 

wait = True
while wait:
        envNum = input("Enter 1 or 2 to select environment used to plan path.txt: ")
        if envNum == '1':
            print("Displaying path.txt in Env 1")
            # ENVIRONMENT 1:  (x of lower left, y of lower left), width, height
            r1 = patch.Rectangle((1, 1), 2, 4)
            r2 = patch.Rectangle((4, 4), 2, 1)
            r3 = patch.Rectangle((4, 1), 2, 1)

            ax.add_patch(r1)
            ax.add_patch(r2)
            ax.add_patch(r3)

            wait = False
            break
        elif envNum == '2':
            print("Displaying path.txt in Env 1")
             #Environment 2:
            r1 = patch.Rectangle((2,0), 2.5, 1)
            r2 = patch.Rectangle((4.5, 1.5), 1, 5)
            r3 = patch.Rectangle((3, 2), 2, 2)
            r4 = patch.Rectangle((1, 3), 1, 3)

            ax.add_patch(r1)
            ax.add_patch(r2)
            ax.add_patch(r3)
            ax.add_patch(r4)

            wait = False
            break
        else:
            print("Invalid input. Enter 1 or 2.")

plt.show()