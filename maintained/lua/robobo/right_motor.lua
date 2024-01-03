function sysCall_init()
    -- do some initialization here
    motor = sim.getObjectHandle("Left_Motor")
    pos_anterior = sim.getJointPosition(motor)
end

function sysCall_actuation()
    -- put your actuation code here
    pos = sim.getJointPosition(motor)
    velocidad = math.floor((pos - pos_anterior) / 0.05 * 180 / math.pi)
    print(velocidad)
    pos_anterior = pos
end
