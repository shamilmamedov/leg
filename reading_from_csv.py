import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def plt_and_save(motors_to_show, subtitle, topic_to_plot, topic_to_plot_ref, fig_to_save, y_label, x_label = 't, sec'):
    '''
    motors_to_show: list of motors to show info about;
    subtitle: str title of figure;
    topic_to_plot: list of the topic to plot;
    topic_to_plot_ref: <...>;
    fig_to_save: name of the fig to save figure in (without extension)
    '''
    fig, ax = plt.subplots(1, len(motors_to_show), constrained_layout=True)
    fig.suptitle(subtitle)
    for motor_to_show in motors_to_show:
        if len(motors_to_show) == 1:
            if topic_to_plot_ref != None:
                ax.plot(t, topic_to_plot[motor_to_show], t, topic_to_plot_ref[motor_to_show])
            else:
                ax.plot(t, topic_to_plot[motor_to_show])
            ax.set_xlabel(x_label)
            ax.set_ylabel(y_label)
            ax.grid()
        else:
            i = motors_to_show.index(motor_to_show)
            if topic_to_plot_ref != None:
                ax[i].plot(t, topic_to_plot[motor_to_show], t, topic_to_plot_ref[motor_to_show])
            else:
                ax[i].plot(t, topic_to_plot[motor_to_show])
            ax[i].set_xlabel(x_label)
            ax[i].set_ylabel(y_label+'_'+str(motor_to_show))
            ax[i].grid()
    plt.savefig("figures/"+fig_to_save+".svg")
    plt.show()


csv_name = 'jumps_A0.1_w6.283185307179586' #withous extension

try:
    path_to_csv = 'logs/'+csv_name +'.csv'
    df = pd.read_csv(path_to_csv, delimiter = ',')
except FileNotFoundError:
    path_to_csv = 'logs/'+csv_name
    df = pd.read_csv(path_to_csv, delimiter = ',')

info_topics = list(df.columns)

# if added in case if some info topics weren't added into the csv file
# qi - angles of the motors
# qi_ref - reference angles of the motors
# dqi - velocities of the motors
# taui - the torques of the motors
#tau1_ref - the reference torques of the motors
if 'time' in info_topics:
    t = np.array(df['time'])
    t = t - t[0] # make the first time occurence equal to zero
    q1 = q2 = q3 = q1_ref = q2_ref = q3_ref = q1_dot = q2_dot = q3_dot = tau1 = tau2 = tau3 = tau1_ref = tau2_ref = tau3_ref =  np.zeros(t.shape)
if 'q1' in info_topics:
    q1 = np.array(df['q1'])
if 'q2' in info_topics:
    q2 = np.array(df['q2'])
if 'q3' in info_topics:
    q3 = np.array(df['q3'])

if 'q1_ref' in info_topics:
    q1_ref = np.array(df['q1_ref'])
if 'q2_ref' in info_topics:
    q2_ref = np.array(df['q2_ref'])
if 'q3_ref' in info_topics:
    q3_ref = np.array(df['q3_ref'])

if 'q1_dot' in info_topics:
    q1_dot = np.array(df['q1_dot'])
if 'q2_dot' in info_topics:
    q2_dot = np.array(df['q2_dot'])
if 'q3_dot' in info_topics:
    q3_dot = np.array(df['q3_dot'])

if 'tau1' in info_topics:
    tau1 = np.array(df['tau1'])
if 'tau2' in info_topics:
    tau2 = np.array(df['tau2'])
if 'tau3' in info_topics:
    tau3 = np.array(df['tau3'])

if 'tau1_ref' in info_topics:
    tau1_ref = np.array(df['tau1_ref'])
if 'tau2_ref' in info_topics:
    tau2_ref = np.array(df['tau2_ref'])
if 'tau3_ref' in info_topics:
    tau3_ref = np.array(df['tau3_ref'])

q = [q1, q2, q3]
q_ref = [q1_ref, q2_ref, q3_ref]
q_dot = [q1_dot, q2_dot, q3_dot]
tau = [tau1, tau2, tau3]
tau_ref = [tau1_ref, tau2_ref, tau3_ref]

motors_to_show = [0,1] # could be any: [0] or [0,2] and so on

# Plot and save
if not os.path.exists('figures'): # directory for storing figures
    os.mkdir('figures')

# Angles
plt_and_save(motors_to_show, r'Angles $q_i$ (rad)', q, q_ref, csv_name+'_angles'+'_motors_'+str(motors_to_show), y_label = 'q')

# # Velocities
plt_and_save(motors_to_show, r'Velocities $\dot{q}_i$ (rad/s)', q_dot, None, csv_name+'_velocities'+'_motors_'+str(motors_to_show), y_label = r'$\dot{q}$')

# # Torques
plt_and_save(motors_to_show, r'Torques $\tau_i$ ($N*m$)', tau, tau_ref, csv_name+'_torques'+'_motors_'+str(motors_to_show), y_label = r'$\tau$')
