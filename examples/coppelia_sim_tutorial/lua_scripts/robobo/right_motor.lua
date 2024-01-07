local sim = require("sim")

function sysCall_init()
    -- do some initialization here
    motor = sim.sim.getObject("../Left_Motor")
    pos_anterior = sim.getJointPosition(motor)
end

function sysCall_actuation()
    -- put your actuation code here
    pos = sim.getJointPosition(motor)
    velocidad = math.floor((pos - pos_anterior) / 0.05 * 180 / math.pi)
    pos_anterior = pos
end
