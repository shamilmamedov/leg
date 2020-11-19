import csv
import numpy as np

t = np.linspace(0, 9, 10)
q = np.random.rand(2,10)
q_dot = np.random.rand(2,10)
tau = np.random.rand(2,10)

A = np.array([5, 0])
omega = np.array([10, 20])
filename = 'logs/jumps_A' + str(A[0]) + "_w" + str(omega[0])

with open(filename, 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
    spamwriter.writerow(['time', 'q1', 'q2', 'q1_dot', 'q2_dot', 'tau1', 'tau2'])
    for k in range(np.shape(t)[0]):
        current_line = [t[k], q[0,k], q[1,k], q_dot[0,k], q_dot[1,k], tau[0,k], tau[1,k]]
        spamwriter.writerow(current_line)