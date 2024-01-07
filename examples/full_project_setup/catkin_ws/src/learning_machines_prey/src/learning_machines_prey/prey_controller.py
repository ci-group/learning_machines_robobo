import random
from dataclasses import dataclass

from robobo_interface import IRobobo
from .stoppable_thread import StoppableThread


@dataclass
class Difficulty:
    maximum_speed: float
    turning_speed: float
    epsilon: float


DIFFICULTIES = {
    "super_easy": Difficulty(
        maximum_speed=5.0,
        turning_speed=5.0,
        epsilon=0.02,
    ),
    "easy": Difficulty(
        maximum_speed=10.0,
        turning_speed=10.0,
        epsilon=0.03,
    ),
    "medium": Difficulty(
        maximum_speed=20.0,
        turning_speed=10.0,
        epsilon=0.06,
    ),
    "hard": Difficulty(
        maximum_speed=40.0,
        turning_speed=20.0,
        epsilon=0.08,
    ),
    "insane": Difficulty(
        maximum_speed=70.0,
        turning_speed=30.0,
        epsilon=0.1,
    ),
}


class Prey(StoppableThread):
    def __init__(
        self,
        robot: IRobobo,
        seed: int = 42,
        difficulty: Difficulty = DIFFICULTIES["medium"],
    ):
        """A prey that can be spawened as a seperate thread.
        Takes a difficulty paramter to control how fast this robot will go.
        Look at DIFFICULTIES for the standard values.
        """
        super(Prey, self).__init__()
        self._robot = robot
        self._seed = seed
        self._difficulty = difficulty

    def _sensor_better_reading(self, sensors_values):
        """
        Normalising simulation sensor reading due to reuse old code
        :param sensors_values:
        :return:
        """
        old_min = 0
        old_max = 0.20
        new_min = 20000
        new_max = 0
        return [
            0
            if value is False
            else (((value - old_min) * (new_max - new_min)) / (old_max - old_min))
            + new_min
            for value in sensors_values
        ]

    def run(self):
        """
        Method that moves the robot.
        It avoids obstacles and with a predefined probability it changes direction
        """
        random.seed = self._seed

        while not self.stopped():
            speed_right = speed_left = self._difficulty.maximum_speed
            if random.random() <= self._difficulty.epsilon:
                speed_right = random.uniform(
                    -self._difficulty.turning_speed, self._difficulty.turning_speed
                )
                speed_left = random.uniform(
                    -self._difficulty.turning_speed, self._difficulty.turning_speed
                )
                for _ in range(3):
                    self._robot.move(left=speed_right, right=speed_left, millis=200)
            self._robot.move(left=speed_right, right=speed_left, millis=200)
            sensors = self._sensor_better_reading(self._robot.read_irs())

            if sum(sensors) != 0:
                index_max_value = sensors.index(max(sensors))
                if index_max_value == 5:
                    # central -> turn
                    if random.random() <= 0.5:
                        while sum(sensors[3:5]) > 500:
                            self._robot.move(left=-10.0, right=20.0, millis=500)
                            sensors = self._sensor_better_reading(
                                self._robot.read_irs()
                            )
                    else:
                        while sum(sensors[6:8]) > 500:
                            self._robot.move(left=20.0, right=-10, millis=500)
                            sensors = self._sensor_better_reading(
                                self._robot.read_irs()
                            )
                elif index_max_value == 3 or index_max_value == 4:
                    # front right right -> go left
                    while sum(sensors[3:5]) > 500:
                        self._robot.move(left=-10.0, right=20.0, millis=200)
                        sensors = self._sensor_better_reading(self._robot.read_irs())
                elif index_max_value == 7 or index_max_value == 6:
                    # front left left -> go right
                    while sum(sensors[6:8]) > 500:
                        self._robot.move(left=20.0, right=-10.0, millis=200)
                        sensors = self._sensor_better_reading(self._robot.read_irs())
