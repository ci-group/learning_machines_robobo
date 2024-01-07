local sim = require("sim")

function sysCall_init()
    -- se ejecuta una vez al inicio de la simulacion
    motor = sim.getObject(".")
    v = 0
end

function sysCall_actuation()
    -- se ejecuta en cada paso de simulacion
    if v > 0 then
        posicion_actual = sim.getJointPosition(motor) * 180 / math.pi
        -- Cuando la diferencia entre la posicion objetivo y la actual sea menor a 2 se detiene la articulacion y se resetean las variables globales
        if (posicion_objetivo - posicion_actual) * parametro < 1 then
            sim.setJointTargetVelocity(motor, 0)
            sim.setInt32Signal(signal, 0)
            v = 0
        end
    end
end

movePanTo = function(parametros, inFloat, inString, inBuffer) -- "s" es un vector de dos elementos: posicion objetivo (en grados) del pan y su velocidad
    parametros[1] = -1 * (parametros[1] - 180)
    -- Se asegura que la posicion objetivo del pan este comprendida entre -160 y 160, y su velocidad entre 0 y 100
    if parametros[1] > 160 then
        posicion_objetivo = 160
    elseif parametros[1] < -160 then
        posicion_objetivo = -160
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
        posicion_inicial = math.floor(sim.getJointPosition(motor) * 180 / math.pi) -- Posicion del pan en el momento de la llamada a la funcion
        -- Comparacion de la posicion objetivo del pan con la posicion actual y asignacion del sentido de giro de la articulacion
        if posicion_objetivo > posicion_inicial then
            parametro = 1
        elseif posicion_objetivo < posicion_inicial then
            parametro = -1
        else
            parametro = 0
        end
        -- Calculo de la velocidad de la articulacion dividiendo el angulo entre el tiempo despejado del polinomio desarrollado en la teoria
        angulo = math.abs(posicion_objetivo - posicion_inicial)
        sim.setJointTargetVelocity(
            motor,
            parametro
                * angulo
                / ((angulo - (-0.000020 * (v ^ 3) + 0.001064 * (v ^ 2) - 0.330338 * v - 0.890735)) / (-0.000031 * (v ^ 3) + 0.003877 * (v ^ 2) + 0.847465 * v + 8.054684))
                * math.pi
                / 180
        )
    end
    return {}, {}, {}, ""
end

readPanPosition = function(inIntegers, inFloats, inStrings, inBuffer)
    posicionPanInt = {} -- Definicion del vector donde se guarda la posicion (en grados) del pan en el momento de llamar a la funcion
    posicionPanFloat = -sim.getJointPosition(motor) * 180 / math.pi
    posicionPanInt[1] = math.floor(posicionPanFloat) + 180
    return posicionPanInt, {}, {}, ""
end
