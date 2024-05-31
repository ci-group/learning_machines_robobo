local sim = require("sim")

function sysCall_init()
    -- do some initialization here
    aceleracion = {} -- Definicion del vector donde se guarda la orientacion del smartphone
    orientacion = {} -- Definicion del vector donde se guarda la orientacion del smartphone
    smartphone = sim.getObject(".")
    smartphone_camera = sim.getObject("./Smartphone_camera")
    pos_anterior = sim.getObjectPosition(smartphone, -1)
    pos_antant = sim.getObjectPosition(smartphone, -1)
end

readOrientationSensor = function(inIntegers, inFloats, inStrings, inBuffer)
    orientacion = sim.getObjectOrientation(smartphone, -1)
    orientacion[1], orientacion[2], orientacion[3] =
        sim.alphaBetaGammaToYawPitchRoll(orientacion[1], orientacion[2], orientacion[3])
    orientacion[1] = orientacion[1] * 180 / math.pi
    orientacion[2] = orientacion[2] * 180 / math.pi
    orientacion[3] = orientacion[3] * 180 / math.pi
    return {}, orientacion, {}, ""
end

readAccelerationSensor = function(inIntegers, inFloats, inStrings, inBuffer)
    pos = sim.getObjectPosition(smartphone, -1)
    aceleracion[1] = pos[1] - 2 * pos_anterior[1] + pos_antant[1]
    aceleracion[2] = pos[2] - 2 * pos_anterior[2] + pos_antant[2]
    aceleracion[3] = pos[3] - 2 * pos_anterior[3] + pos_antant[3]
    pos_antant = pos_anterior
    pos_anterior = pos
    return {}, aceleracion, {}, ""
end
