#!/usr/bin/env python3
# from _future_ import print_function

from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes
from controller import *


import time
import numpy as np
import pandas
import robobo
import cv2
import sys
import signal
import prey
import csv
coef1 = 10
coef2 = 50


def terminate_program(signal_number, frame):
    print("Ctrl-C received, terminating program")
    sys.exit(1)


def fitness(controller):
    signal.signal(signal.SIGINT, terminate_program)

    # rob = robobo.HardwareRobobo(camera=True).connect(address="192.168.2.28")
    rob = robobo.SimulationRobobo().connect(address='127.0.0.1', port=19997)

    rob.play_simulation()

    # Following code moves the robot
    with open('output.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(
            ['time', 'x', 'y', 'z', 'sensor1', 'sensor2', 'sensor3', 'sensor4', 'sensor5', 'sensor6', 'sensor7', 'sensor8'])

        rob.play_simulation()

        end = rob.get_sim_time() + 60 * 1000
        now = rob.get_sim_time()

        while now < end:
            position = rob.position()
            irs = np.array(rob.read_irs())

            now = rob.get_sim_time()

            file.write("{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                now, position[0], position[1],
                position[2], irs[0], irs[1], irs[2],
                irs[3], irs[4], irs[5],
                irs[6], irs[7]))
            # writer.writerow(time() + rob.position() + np.array(rob.read_irs()))
            left, right = controller.act(irs)
            rob.move(left, right, 2000)

    dataframe = pandas.read_csv('output.csv', sep=',')
    xval = dataframe.x.copy()[1:]
    yval = dataframe.y.copy()[1:]
    zval = dataframe.z.copy()[1:]

    dataframe["Euclid"] = 0
    dataframe["Euclid"] = np.sqrt(
        (dataframe.x[0:(len(dataframe.x) - 1)] - np.array(xval)) ** 2 +
        (dataframe.y[0:(len(dataframe.y) - 1)] - np.array(yval)) ** 2 +
        (dataframe.z[0:(len(dataframe.z) - 1)] - np.array(zval)) ** 2
    )
    dataframe["Euclidean"] = 0
    dataframe["Euclidean"][1:] = dataframe.Euclid[0:(len(dataframe.Euclid) - 1)].cumsum()
    dataframe["Rel_Diff"] = 'False'
    dataframe["Rel_Diff"][dataframe.sensor4[dataframe.sensor4 != 'False'].index] = 0.2 * (
                dataframe.sensor4[dataframe.sensor4 != 'False'].astype(float) + dataframe.sensor5[
            dataframe.sensor5 != 'False'].astype(float) + dataframe.sensor6[dataframe.sensor6 != 'False'].astype(
            float) + dataframe.sensor7[dataframe.sensor7 != 'False'].astype(float) + dataframe.sensor8[
                    dataframe.sensor8 != 'False'].astype(float))
    dataframe["Rel_Diff"][dataframe.Rel_Diff == 'False'] = 100
    dataframe["Rel_Diff"] = dataframe.Rel_Diff.astype(float).diff()
    dataframe["Rel_Diff"][dataframe.Rel_Diff.isnull() == True] = 0
    counter = 0
    for j in range(len(dataframe["Rel_Diff"])):
        if j < (len(dataframe["Rel_Diff"]) - 4):
            if np.abs(dataframe.Rel_Diff[j]) > 0.03 and np.abs(dataframe.Rel_Diff.diff()[j + 2]) < 0.01 and np.abs(
                    dataframe.Rel_Diff.diff()[j + 4]) < 0.001:
                counter += 1
                print(j)
    a = dataframe.Rel_Diff.copy()
    round(a[0:18], 6)
    round(a.diff()[0:20], 6)
    fitness = coef1 * dataframe.Euclidean[len(dataframe.Euclidean) - 1] + (coef2 / (counter + 1))
    # Following code moves the phone stand
    # rob.set_phone_pan(343, 100)
    # rob.set_phone_tilt(109, 100)
    # # time.sleep(1)
    # rob.set_phone_pan(11, 100)
    # rob.set_phone_tilt(26, 100)

    # Following code makes the robot talk and be emotional
    # rob.set_emotion('happy')
    # rob.talk('Hi, my name is Robobo')
    # rob.sleep(1)
    # rob.set_emotion('sad')

    # Following code gets an image from the camera
    image = rob.get_image_front()
    cv2.imwrite("test_pictures.png", image)

    time.sleep(0.1)

    # IR reading
    # for i in range(100):
    #     print("ROB Irs: {}".format(np.log(np.array(rob.read_irs())) / 10))
    #     time.sleep(0.1)

    # pause the simulation and read the collected food
    rob.pause_simulation()

    # Stopping the simualtion resets the environment
    rob.stop_world()
    return fitness


if __name__ == "__main__":
    gene = [0.1] * 80 + [0.2] * 10 + [0.3] * 20 + [0.4] * 2
    c = Controller(gene)
    fitness(c)