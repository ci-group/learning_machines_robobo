#!/usr/bin/env python2
import time
import robobo

if __name__ == "__main__":
    robs = [
        robobo.HardwareRobobo(),
        robobo.SimulationRobobo(),
    ]
    robs[0].connect(address="192.168.1.247")
    robs[1].connect(address='192.168.1.204', port=19999)
    
    for i, rob in enumerate(robs):
        print('sending commands to robot {}'.format(i))
        rob.set_emotion('sad')
        rob.move(5, -5, 2000)
        rob.talk('Hi, my name is Robobo {}'.format(i))
        rob.sleep(1)
        rob.set_emotion('happy')

    time.sleep(5)
    for rob in robs:
        rob.move(-5, 5, 2000)
