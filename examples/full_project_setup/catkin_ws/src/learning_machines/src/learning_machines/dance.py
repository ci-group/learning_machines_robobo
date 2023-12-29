from robobo_interface import IRobobo, Emotion, LedId, LedColor, SoundEmotion


def test_emotions(rob: IRobobo):
    rob.set_emotion(Emotion.HAPPY)
    rob.talk("Hello")
    rob.play_emotion_sound(SoundEmotion.PURR)
    rob.set_led(LedId.FRONTCENTER, LedColor.GREEN)


def test_move_and_wheel_reset(rob: IRobobo):
    rob.move_blocking(100, 100, 1000)
    print("before reset: ", rob.read_wheels())
    rob.reset_wheels()
    rob.sleep(1)
    print("after reset: ", rob.read_wheels())


def test_sensors(rob: IRobobo):
    # print("IRS data: ", rob.read_irs())
    print("Image data: ", rob.get_image_front())
    # print("Phone pan: ", rob.read_phone_pan())
    # print("Phone tilt: ", rob.read_phone_tilt())
    # print("Current acceleration: ", rob.read_accel())
    # print("Current orientation: ", rob.read_orientation())


def test_phone_movement(rob: IRobobo):
    rob.set_phone_pan_blocking(20, 100)
    print("Phone pan after move to 20: ", rob.read_phone_pan())
    rob.set_pan_exact(120)
    print("Phone pan after exact to 20: ", rob.read_phone_pan())
    rob.set_phone_tilt_blocking(50, 100)
    print("Phone tilt: ", rob.read_phone_tilt())


def dance(rob: IRobobo):
    # test_emotions(rob)
    test_sensors(rob)
    # test_move_and_wheel_reset(rob)
    # test_phone_movement(rob)
