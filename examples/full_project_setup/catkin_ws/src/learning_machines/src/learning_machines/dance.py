from robobo_interface import IRobobo, Emotion


def dance(rob: IRobobo):
    rob.set_emotion(Emotion.HAPPY)
    rob.set_phone_pan(10, 10, 1)
    rob.move(10, 10, 1000, 1)
