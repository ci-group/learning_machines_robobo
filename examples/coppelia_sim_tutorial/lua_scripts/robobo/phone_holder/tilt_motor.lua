local sim = require("sim")

function sysCall_init()
    -- se ejecuta una vez al inicio de la simulacion
    motor = sim.getObject(".")
    v = 0
end

function sysCall_actuation()
    -- se ejecuta en cada paso de simulacion
    if v > 0 then
        posicion_actual = sim.getJointPosition(motor) * 180 / math.pi + 70
        -- Cuando la diferencia entre la posicion objetivo y la actual sea menor a 2 se detiene la articulacion y se resetean las variables globales
        if (posicion_objetivo - posicion_actual) * parametro < 1 then
            sim.setJointTargetVelocity(motor, 0)
            sim.setInt32Signal(signal, 0)
            sim.setObjectInt32Param(motor, sim.jointintparam_ctrl_enabled, 1)
            sim.setJointTargetPosition(motor, (posicion_actual - 70) * math.pi / 180)
            v = 0
        end
    end
end

moveTiltTo = function(parametros, inFloat, inString, inBuffer) -- "parametros" es un vector de dos elementos: posicion objetivo (en grados) del tilt y su velocidad
    -- Se asegura que la posicion objetivo del tilt este comprendida entre 0 y 105, y su velocidad entre 0 y 100
    if parametros[1] > 105 then
        posicion_objetivo = 105
    elseif parametros[1] < 5 then
        posicion_objetivo = 5
    else
        posicion_objetivo = parametros[1]
    end
    if parametros[2] > 100 then
        v = 100
    elseif parametros[2] < 0 then
        v = 0
    else
        v = parametros[2]
    end
    if v > 0 then
        signal = inString[1]
        sim.setInt32Signal(signal, 1) -- Definicion de una variable global que bloquea la funcion (1) o la desbloquea (0)
        posicion_inicial = math.floor(sim.getJointPosition(motor) * 180 / math.pi + 70) -- Posicion del tilt en el momento de la llamada a la funcion
        -- Comparacion de la posicion objetivo del tilt con la posicion inicial y asignacion del sentido de giro de la articulacion
        if posicion_objetivo > posicion_inicial then
            parametro = 1
        elseif posicion_objetivo < posicion_inicial then
            parametro = -1
        else
            parametro = 0
        end
        -- Calculo de la velocidad de la articulacion dividiendo el angulo entre el tiempo despejado del polinomio desarrollado en la teoria
        angulo = math.abs(posicion_objetivo - posicion_inicial)
        sim.setObjectInt32Param(motor, sim.jointintparam_ctrl_enabled, 0)
        sim.setJointTargetVelocity(
            motor,
            parametro
                * angulo
                / ((angulo - (-0.000088 * (v ^ 3) + 0.012325 * (v ^ 2) - 0.549530 * v + 4.737946)) / (0.000014 * (v ^ 3) - 0.002598 * (v ^ 2) + 0.480940 * v + 3.181534))
                * math.pi
                / 180
        )
    end
    return {}, {}, {}, ""
end

readTiltPosition = function(inIntegers, inFloats, inStrings, inBuffer)
    posicionTiltInt = {} -- Definicion del vector donde se guarda la posicion (en grados) del tilt en el momento de llamar a la funcion
    posicionTiltFloat = sim.getJointPosition(motor) * 180 / math.pi + 70
    posicionTiltInt[1] = math.floor(posicionTiltFloat)
    return posicionTiltInt, {}, {}, ""
end
