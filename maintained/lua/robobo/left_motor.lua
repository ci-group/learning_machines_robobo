local sim = require("sim")

function sysCall_init()
    -- do some initialization here
    motorI = sim.getObject("../Left_Motor")
    motorD = sim.getObject("../Right_Motor")
    d = 0
    pos_inicial_I = 0
    pos_inicial_D = 0
end

function sysCall_actuation()
    -- put your actuation code here
    if d > 0 then
        -- Cuando se supere el tiempo de duracion del movimiento de las ruedas, se paran los joints y se resetean las variables globales
        if (sim.getSimulationTime() - t_inicial) >= d then
            sim.setJointTargetVelocity(motorI, 0)
            sim.setJointTargetVelocity(motorD, 0)
            sim.setInt32Signal(signal, 0)
            d = 0
            vI = 0
            vD = 0
        end
    end
end

moveWheelsByTime = function(velocidad, tiempo, inString, inBuffer) -- "velocidad" es un vector de dos elementos: velocidades de la rueda derecha e izquierda
    d = tiempo[1] -- Definicion de una variable que guarda el tiempo que se moveran las ruedas
    t_inicial = sim.getSimulationTime() -- Definicion de una variable que guarda el instante en el que se llama a la funcion
    signal = inString[1]
    sim.setInt32Signal(signal, 1) -- Definicion de una variable global que bloquea la funcion (1) o la desbloquea (0)
    -- Se asegura que los argumentos de velocidad recibidos tienen un valor entre -100 y 100
    if velocidad[2] > 100 then
        vI = 100
    elseif velocidad[2] < -100 then
        vI = -100
    else
        vI = velocidad[2]
    end
    if velocidad[1] > 100 then
        vD = 100
    elseif velocidad[1] < -100 then
        vD = -100
    else
        vD = velocidad[1]
    end
    --Calculo de la velocidad de la rueda izquierda dividiendo los grados calculados con el polinomio entre el tiempo asignado
    if vI > 0 then
        sim.setJointTargetVelocity(
            motorI,
            (
                (0.000001646 * (vI ^ 3) - 0.00285 * (vI ^ 2) + 6.649 * vI + 51.14)
                + (-0.0002912 * (vI ^ 3) + 0.04647 * (vI ^ 2) - 1.339 * vI - 12.25)
                    / d
            )
                * math.pi
                / 180
        )
    elseif vI < 0 then
        sim.setJointTargetVelocity(
            motorI,
            -(
                    (
                        0.000001646 * math.abs(vI ^ 3)
                        - 0.00285 * (vI ^ 2)
                        + 6.649 * math.abs(vI)
                        + 51.14
                    )
                    + (
                            -0.0002912 * math.abs(vI ^ 3)
                            + 0.04647 * (vI ^ 2)
                            - 1.339 * math.abs(vI)
                            - 12.25
                        )
                        / d
                )
                * math.pi
                / 180
        )
    else
        sim.setJointTargetVelocity(motorI, 0)
    end
    --Calculo de la velocidad de la rueda izquierda dividiendo los grados calculados con el polinomio entre el tiempo asignado
    if vD > 0 then
        sim.setJointTargetVelocity(
            motorD,
            (
                (0.000001646 * (vD ^ 3) - 0.00285 * (vD ^ 2) + 6.649 * vD + 51.14)
                + (-0.0002912 * (vD ^ 3) + 0.04647 * (vD ^ 2) - 1.339 * vD - 12.25)
                    / d
            )
                * math.pi
                / 180
        )
    elseif vD < 0 then
        sim.setJointTargetVelocity(
            motorD,
            -(
                    (
                        0.000001646 * math.abs(vD ^ 3)
                        - 0.00285 * (vD ^ 2)
                        + 6.649 * math.abs(vD)
                        + 51.14
                    )
                    + (
                            -0.0002912 * math.abs(vD ^ 3)
                            + 0.04647 * (vD ^ 2)
                            - 1.339 * math.abs(vD)
                            - 12.25
                        )
                        / d
                )
                * math.pi
                / 180
        )
    else
        sim.setJointTargetVelocity(motorD, 0)
    end
    return {}, {}, {}, ""
end

moveWheelsByDegrees = function(parametros, inFloats, rueda, inBuffer) -- "parametros" es un vector de dos elementos:
    --velocidad de la/s rueda/s y los grados a recorrer por la/s misma/s
    --"rueda" es un string que indica la/s rueda/s que se quiere/n mover
    t_inicial = sim.getSimulationTime()
    sim.setInt32Signal("Bloqueado", 1)
    --El tiempo se obtiene despejando la variable t del polinomio desarrollado en el apartado 7.2.2 de la memoria
    d = math.max(
        (
            parametros[1]
            - (
                -0.000325 * (math.abs(parametros[2]) ^ 3)
                + 0.042847 * (math.abs(parametros[2]) ^ 2)
                - 2.064049 * (math.abs(parametros[2]))
                - 17.697271
            )
        )
            / (
                -0.000046 * (math.abs(parametros[2]) ^ 3)
                + 0.005219 * (math.abs(parametros[2]) ^ 2)
                + 6.357077 * (math.abs(parametros[2]))
                + 51.366765
            ),
        0
    )

    --Se asegura que los argumentos de velocidad recibidos tienen un valor entre -100 y 100 y asigna para cada rueda
    if parametros[2] > 100 then
        if rueda[1] == "left" then
            vI = 100
        elseif rueda[1] == "right" then
            vD = 100
        else
            vI = 100
            vD = 100
        end
    elseif parametros[2] < -100 then
        if rueda[1] == "left" then
            vI = -100
        elseif rueda[1] == "right" then
            vD = -100
        else
            vI = -100
            vD = -100
        end
    else
        if rueda[1] == "left" then
            vI = parametros[2]
        elseif rueda[1] == "right" then
            vD = parametros[2]
        else
            vI = parametros[2]
            vD = parametros[2]
        end
    end
    --Calculo de la velocidad de la rueda izquierda dividiendo los grados calculados con el polinomio entre el tiempo asignado
    if vI > 0 then
        sim.setJointTargetVelocity(
            motorI,
            (
                (0.000001646 * (vI ^ 3) - 0.00285 * (vI ^ 2) + 6.649 * vI + 51.14)
                + (-0.0002912 * (vI ^ 3) + 0.04647 * (vI ^ 2) - 1.339 * vI - 12.25)
                    / d
            )
                * math.pi
                / 180
        )
    elseif vI < 0 then
        sim.setJointTargetVelocity(
            motorI,
            -(
                    (
                        0.000001646 * math.abs(vI ^ 3)
                        - 0.00285 * (vI ^ 2)
                        + 6.649 * math.abs(vI)
                        + 51.14
                    )
                    + (
                            -0.0002912 * math.abs(vI ^ 3)
                            + 0.04647 * (vI ^ 2)
                            - 1.339 * math.abs(vI)
                            - 12.25
                        )
                        / d
                )
                * math.pi
                / 180
        )
    else
        sim.setJointTargetVelocity(motorI, 0)
    end
    --Calculo de la velocidad de la rueda izquierda dividiendo los grados calculados con el polinomio entre el tiempo asignado
    if vD > 0 then
        sim.setJointTargetVelocity(
            motorD,
            (
                (0.000001646 * (vD ^ 3) - 0.00285 * (vD ^ 2) + 6.649 * vD + 51.14)
                + (-0.0002912 * (vD ^ 3) + 0.04647 * (vD ^ 2) - 1.339 * vD - 12.25)
                    / d
            )
                * math.pi
                / 180
        )
    elseif vD < 0 then
        sim.setJointTargetVelocity(
            motorD,
            -(
                    (
                        0.000001646 * math.abs(vD ^ 3)
                        - 0.00285 * (vD ^ 2)
                        + 6.649 * math.abs(vD)
                        + 51.14
                    )
                    + (
                            -0.0002912 * math.abs(vD ^ 3)
                            + 0.04647 * (vD ^ 2)
                            - 1.339 * math.abs(vD)
                            - 12.25
                        )
                        / d
                )
                * math.pi
                / 180
        )
    else
        sim.setJointTargetVelocity(motorD, 0)
    end
    return {}, {}, {}, ""
end

readWheels = function(inIntegers, inFloats, rueda, inBuffer)
    medidasRuedas = {} -- Definicion del vector donde se guarda la posicion y velocidad de las ruedas en el momento de llamar a la funcion
    -- {posicionRuedaI,posicionRuedaD,velocidadRuedaI,velocidadRuedaD}
    -- Para leer los encoders de las ruedas se calcula la diferencia entre la posicion actual y la posicion referencia
    medidasRuedas[1] =
        math.floor(sim.getJointPosition(motorD) * 180 / math.pi - pos_inicial_D)
    medidasRuedas[2] =
        math.floor(sim.getJointPosition(motorI) * 180 / math.pi - pos_inicial_I)
    medidasRuedas[3] = vI
    medidasRuedas[4] = vD
    return medidasRuedas, {}, {}, ""
end

resetWheelEncoders = function(inIntegers, inFloats, inStrings, inBuffer)
    -- Al resetear los encoders se hace que la posicion de referencia sea la posicion actual de la rueda
    pos_inicial_I = math.floor(sim.getJointPosition(motorI) * 180 / math.pi)
    pos_inicial_D = math.floor(sim.getJointPosition(motorD) * 180 / math.pi)
    return {}, {}, {}, ""
end
