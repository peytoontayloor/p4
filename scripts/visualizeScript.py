
"""
 Enter 1 (Car) or 2 (Pendulum) in command line to visualize path.txt 
"""
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import sys

data = numpy.loadtxt('scripts/path.txt')
fig, ax = plt.subplots()
ax.plot(data[:, 0],data[:, 1],'.-')

wait = True
while wait:
        
        envNum = input("Enter 1 (Car) or 2 (Pendulum) to visualize path.txt: ")
        if envNum == '1':
            ax.set_xlim(-5, 5)  
            ax.set_ylim(-10, 10)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_aspect('equal') 
            print("Displaying path.txt, Car robot")

            # STREET ENVIRONMENT:  (x of lower left, y of lower left), width, height
            r1 = patch.Rectangle((-5, -10), 10, 2)
            r2 = patch.Rectangle((-5, -4), 4, 8)
            r3 = patch.Rectangle((2, -4), 3, 8)
            r4 = patch.Rectangle((-5, 6), 10, 2)

            ax.add_patch(r1)
            ax.add_patch(r2)
            ax.add_patch(r3)
            ax.add_patch(r4)

            wait = False
            break
        elif envNum == '2':
            print("Displaying path.txt, Pendulum robot")

            ax.set_xlim(-5, 5)
            ax.set_ylim(-10, 10) 
            ax.set_xlabel('theta')
            ax.set_ylabel('angular velocity') 
            
            wait = False
            break
        else:
            print("Invalid input. Enter 1 or 2.")

plt.savefig('4b-plots/p4_vis.png')
plt.show()