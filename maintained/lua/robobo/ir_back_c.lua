local sim = require("sim")

function sysCall_init()
    IR_Back_L = sim.getObject("../IR_Back_L")
    IR_Back_L2 = sim.getObject("../IR_Back_L2")
    IR_Back_L3 = sim.getObject("../IR_Back_L3")
    IR_Back_L4 = sim.getObject("../IR_Back_L4")
    IR_Back_L5 = sim.getObject("../IR_Back_L5")
    IR_Back_L6 = sim.getObject("../IR_Back_L6")
    IR_Back_L7 = sim.getObject("../IR_Back_L7")
    IR_Back_L8 = sim.getObject("../IR_Back_L8")
    IR_Back_L9 = sim.getObject("../IR_Back_L9")
    IR_Back_L10 = sim.getObject("../IR_Back_L10")
    IR_Back_L11 = sim.getObject("../IR_Back_L11")
    IR_Back_L12 = sim.getObject("../IR_Back_L12")
    IR_Back_L13 = sim.getObject("../IR_Back_L13")
    IR_Back_L14 = sim.getObject("../IR_Back_L14")
    IR_Back_L15 = sim.getObject("../IR_Back_L15")
    IR_Back_L16 = sim.getObject("../IR_Back_L16")
    IR_Back_R = sim.getObject("../IR_Back_R")
    IR_Back_R2 = sim.getObject("../IR_Back_R2")
    IR_Back_R3 = sim.getObject("../IR_Back_R3")
    IR_Back_R4 = sim.getObject("../IR_Back_R4")
    IR_Back_R5 = sim.getObject("../IR_Back_R5")
    IR_Back_R6 = sim.getObject("../IR_Back_R6")
    IR_Back_R7 = sim.getObject("../IR_Back_R7")
    IR_Back_R8 = sim.getObject("../IR_Back_R8")
    IR_Back_R9 = sim.getObject("../IR_Back_R9")
    IR_Back_R10 = sim.getObject("../IR_Back_R10")
    IR_Back_R11 = sim.getObject("../IR_Back_R11")
    IR_Back_R12 = sim.getObject("../IR_Back_R12")
    IR_Back_R13 = sim.getObject("../IR_Back_R13")
    IR_Back_R14 = sim.getObject("../IR_Back_R14")
    IR_Back_R15 = sim.getObject("../IR_Back_R15")
    IR_Back_R16 = sim.getObject("../IR_Back_R16")
    IR_Front_L = sim.getObject("../IR_Front_L")
    IR_Front_L2 = sim.getObject("../IR_Front_L2")
    IR_Front_L3 = sim.getObject("../IR_Front_L3")
    IR_Front_L4 = sim.getObject("../IR_Front_L4")
    IR_Front_L5 = sim.getObject("../IR_Front_L5")
    IR_Front_L6 = sim.getObject("../IR_Front_L6")
    IR_Front_L7 = sim.getObject("../IR_Front_L7")
    IR_Front_L8 = sim.getObject("../IR_Front_L8")
    IR_Front_L9 = sim.getObject("../IR_Front_L9")
    IR_Front_L10 = sim.getObject("../IR_Front_L10")
    IR_Front_L11 = sim.getObject("../IR_Front_L11")
    IR_Front_L12 = sim.getObject("../IR_Front_L12")
    IR_Front_L13 = sim.getObject("../IR_Front_L13")
    IR_Front_L14 = sim.getObject("../IR_Front_L14")
    IR_Front_L15 = sim.getObject("../IR_Front_L15")
    IR_Front_L16 = sim.getObject("../IR_Front_L16")
    IR_Front_R = sim.getObject("../IR_Front_R")
    IR_Front_R2 = sim.getObject("../IR_Front_R2")
    IR_Front_R3 = sim.getObject("../IR_Front_R3")
    IR_Front_R4 = sim.getObject("../IR_Front_R4")
    IR_Front_R5 = sim.getObject("../IR_Front_R5")
    IR_Front_R6 = sim.getObject("../IR_Front_R6")
    IR_Front_R7 = sim.getObject("../IR_Front_R7")
    IR_Front_R8 = sim.getObject("../IR_Front_R8")
    IR_Front_R9 = sim.getObject("../IR_Front_R9")
    IR_Front_R10 = sim.getObject("../IR_Front_R10")
    IR_Front_R11 = sim.getObject("../IR_Front_R11")
    IR_Front_R12 = sim.getObject("../IR_Front_R12")
    IR_Front_R13 = sim.getObject("../IR_Front_R13")
    IR_Front_R14 = sim.getObject("../IR_Front_R14")
    IR_Front_R15 = sim.getObject("../IR_Front_R15")
    IR_Front_R16 = sim.getObject("../IR_Front_R16")
    IR_Front_C = sim.getObject("../IR_Front_C")
    IR_Front_C2 = sim.getObject("../IR_Front_C2")
    IR_Front_C3 = sim.getObject("../IR_Front_C3")
    IR_Front_C4 = sim.getObject("../IR_Front_C4")
    IR_Front_C5 = sim.getObject("../IR_Front_C5")
    IR_Front_C6 = sim.getObject("../IR_Front_C6")
    IR_Front_C7 = sim.getObject("../IR_Front_C7")
    IR_Front_C8 = sim.getObject("../IR_Front_C8")
    IR_Front_C9 = sim.getObject("../IR_Front_C9")
    IR_Front_C10 = sim.getObject("../IR_Front_C10")
    IR_Front_C11 = sim.getObject("../IR_Front_C11")
    IR_Front_C12 = sim.getObject("../IR_Front_C12")
    IR_Front_C13 = sim.getObject("../IR_Front_C13")
    IR_Front_C14 = sim.getObject("../IR_Front_C14")
    IR_Front_C15 = sim.getObject("../IR_Front_C15")
    IR_Front_C16 = sim.getObject("../IR_Front_C16")
    IR_Front_RR = sim.getObject("../IR_Front_RR")
    IR_Front_RR2 = sim.getObject("../IR_Front_RR2")
    IR_Front_RR3 = sim.getObject("../IR_Front_RR3")
    IR_Front_RR4 = sim.getObject("../IR_Front_RR4")
    IR_Front_RR5 = sim.getObject("../IR_Front_RR5")
    IR_Front_RR6 = sim.getObject("../IR_Front_RR6")
    IR_Front_RR7 = sim.getObject("../IR_Front_RR7")
    IR_Front_RR8 = sim.getObject("../IR_Front_RR8")
    IR_Front_RR9 = sim.getObject("../IR_Front_RR9")
    IR_Front_RR10 = sim.getObject("../IR_Front_RR10")
    IR_Front_RR11 = sim.getObject("../IR_Front_RR11")
    IR_Front_RR12 = sim.getObject("../IR_Front_RR12")
    IR_Front_RR13 = sim.getObject("../IR_Front_RR13")
    IR_Front_RR14 = sim.getObject("../IR_Front_RR14")
    IR_Front_RR15 = sim.getObject("../IR_Front_RR15")
    IR_Front_RR16 = sim.getObject("../IR_Front_RR16")
    IR_Back_C = sim.getObject("../IR_Back_C")
    IR_Back_C2 = sim.getObject("../IR_Back_C2")
    IR_Back_C3 = sim.getObject("../IR_Back_C3")
    IR_Back_C4 = sim.getObject("../IR_Back_C4")
    IR_Back_C5 = sim.getObject("../IR_Back_C5")
    IR_Back_C6 = sim.getObject("../IR_Back_C6")
    IR_Back_C7 = sim.getObject("../IR_Back_C7")
    IR_Back_C8 = sim.getObject("../IR_Back_C8")
    IR_Back_C9 = sim.getObject("../IR_Back_C9")
    IR_Back_C10 = sim.getObject("../IR_Back_C10")
    IR_Back_C11 = sim.getObject("../IR_Back_C11")
    IR_Back_C12 = sim.getObject("../IR_Back_C12")
    IR_Back_C13 = sim.getObject("../IR_Back_C13")
    IR_Back_C14 = sim.getObject("../IR_Back_C14")
    IR_Back_C15 = sim.getObject("../IR_Back_C15")
    IR_Back_C16 = sim.getObject("../IR_Back_C16")
    IR_Front_LL = sim.getObject("../IR_Front_LL")
    IR_Front_LL2 = sim.getObject("../IR_Front_LL2")
    IR_Front_LL3 = sim.getObject("../IR_Front_LL3")
    IR_Front_LL4 = sim.getObject("../IR_Front_LL4")
    IR_Front_LL5 = sim.getObject("../IR_Front_LL5")
    IR_Front_LL6 = sim.getObject("../IR_Front_LL6")
    IR_Front_LL7 = sim.getObject("../IR_Front_LL7")
    IR_Front_LL8 = sim.getObject("../IR_Front_LL8")
    IR_Front_LL9 = sim.getObject("../IR_Front_LL9")
    IR_Front_LL10 = sim.getObject("../IR_Front_LL10")
    IR_Front_LL11 = sim.getObject("../IR_Front_LL11")
    IR_Front_LL12 = sim.getObject("../IR_Front_LL12")
    IR_Front_LL13 = sim.getObject("../IR_Front_LL13")
    IR_Front_LL14 = sim.getObject("../IR_Front_LL14")
    IR_Front_LL15 = sim.getObject("../IR_Front_LL15")
    IR_Front_LL16 = sim.getObject("../IR_Front_LL16")
    distancia = {} -- Definicion del vector que guardar los valores de las 16 distancias leidas por los 16 sensores de proximidad que forman los sensores infrarrojos implementados
    IR = {} -- Definicion del vector que guarda los valores de intensidad de los 8 sensores infrarrojos implementados en el modelo
    -- Definicion de los coeficientes y exponentes optimizados para ajustar el polinomio que calcule la intensidad detectada por el sensor IR a partir de las distancias
    a = 0.1288
    b = -1.7887
end

readAllIRSensor = function(inIntegers, inFloats, inStrings, inBuffer)
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    local _res
    if sim.readProximitySensor(IR_Back_L) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Back_L)
    end
    if sim.readProximitySensor(IR_Back_L2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Back_L2)
    end
    if sim.readProximitySensor(IR_Back_L3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Back_L3)
    end
    if sim.readProximitySensor(IR_Back_L4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Back_L4)
    end
    if sim.readProximitySensor(IR_Back_L5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Back_L5)
    end
    if sim.readProximitySensor(IR_Back_L6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Back_L6)
    end
    if sim.readProximitySensor(IR_Back_L7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Back_L7)
    end
    if sim.readProximitySensor(IR_Back_L8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Back_L8)
    end
    if sim.readProximitySensor(IR_Back_L9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Back_L9)
    end
    if sim.readProximitySensor(IR_Back_L10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Back_L10)
    end
    if sim.readProximitySensor(IR_Back_L11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Back_L11)
    end
    if sim.readProximitySensor(IR_Back_L12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Back_L12)
    end
    if sim.readProximitySensor(IR_Back_L13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Back_L13)
    end
    if sim.readProximitySensor(IR_Back_L14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Back_L14)
    end
    if sim.readProximitySensor(IR_Back_L15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Back_L15)
    end
    if sim.readProximitySensor(IR_Back_L16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Back_L16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[1] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Back_R) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Back_R)
    end
    if sim.readProximitySensor(IR_Back_R2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Back_R2)
    end
    if sim.readProximitySensor(IR_Back_R3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Back_R3)
    end
    if sim.readProximitySensor(IR_Back_R4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Back_R4)
    end
    if sim.readProximitySensor(IR_Back_R5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Back_R5)
    end
    if sim.readProximitySensor(IR_Back_R6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Back_R6)
    end
    if sim.readProximitySensor(IR_Back_R7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Back_R7)
    end
    if sim.readProximitySensor(IR_Back_R8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Back_R8)
    end
    if sim.readProximitySensor(IR_Back_R9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Back_R9)
    end
    if sim.readProximitySensor(IR_Back_R10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Back_R10)
    end
    if sim.readProximitySensor(IR_Back_R11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Back_R11)
    end
    if sim.readProximitySensor(IR_Back_R12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Back_R12)
    end
    if sim.readProximitySensor(IR_Back_R13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Back_R13)
    end
    if sim.readProximitySensor(IR_Back_R14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Back_R14)
    end
    if sim.readProximitySensor(IR_Back_R15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Back_R15)
    end
    if sim.readProximitySensor(IR_Back_R16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Back_R16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[2] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Front_L) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Front_L)
    end
    if sim.readProximitySensor(IR_Front_L2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Front_L2)
    end
    if sim.readProximitySensor(IR_Front_L3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Front_L3)
    end
    if sim.readProximitySensor(IR_Front_L4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Front_L4)
    end
    if sim.readProximitySensor(IR_Front_L5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Front_L5)
    end
    if sim.readProximitySensor(IR_Front_L6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Front_L6)
    end
    if sim.readProximitySensor(IR_Front_L7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Front_L7)
    end
    if sim.readProximitySensor(IR_Front_L8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Front_L8)
    end
    if sim.readProximitySensor(IR_Front_L9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Front_L9)
    end
    if sim.readProximitySensor(IR_Front_L10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Front_L10)
    end
    if sim.readProximitySensor(IR_Front_L11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Front_L11)
    end
    if sim.readProximitySensor(IR_Front_L12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Front_L12)
    end
    if sim.readProximitySensor(IR_Front_L13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Front_L13)
    end
    if sim.readProximitySensor(IR_Front_L14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Front_L14)
    end
    if sim.readProximitySensor(IR_Front_L15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Front_L15)
    end
    if sim.readProximitySensor(IR_Front_L16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Front_L16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[3] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Front_R) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Front_R)
    end
    if sim.readProximitySensor(IR_Front_R2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Front_R2)
    end
    if sim.readProximitySensor(IR_Front_R3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Front_R3)
    end
    if sim.readProximitySensor(IR_Front_R4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Front_R4)
    end
    if sim.readProximitySensor(IR_Front_R5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Front_R5)
    end
    if sim.readProximitySensor(IR_Front_R6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Front_R6)
    end
    if sim.readProximitySensor(IR_Front_R7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Front_R7)
    end
    if sim.readProximitySensor(IR_Front_R8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Front_R8)
    end
    if sim.readProximitySensor(IR_Front_R9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Front_R9)
    end
    if sim.readProximitySensor(IR_Front_R10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Front_R10)
    end
    if sim.readProximitySensor(IR_Front_R11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Front_R11)
    end
    if sim.readProximitySensor(IR_Front_R12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Front_R12)
    end
    if sim.readProximitySensor(IR_Front_R13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Front_R13)
    end
    if sim.readProximitySensor(IR_Front_R14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Front_R14)
    end
    if sim.readProximitySensor(IR_Front_R15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Front_R15)
    end
    if sim.readProximitySensor(IR_Front_R16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Front_R16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[4] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Front_C) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Front_C)
    end
    if sim.readProximitySensor(IR_Front_C2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Front_C2)
    end
    if sim.readProximitySensor(IR_Front_C3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Front_C3)
    end
    if sim.readProximitySensor(IR_Front_C4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Front_C4)
    end
    if sim.readProximitySensor(IR_Front_C5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Front_C5)
    end
    if sim.readProximitySensor(IR_Front_C6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Front_C6)
    end
    if sim.readProximitySensor(IR_Front_C7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Front_C7)
    end
    if sim.readProximitySensor(IR_Front_C8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Front_C8)
    end
    if sim.readProximitySensor(IR_Front_C9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Front_C9)
    end
    if sim.readProximitySensor(IR_Front_C10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Front_C10)
    end
    if sim.readProximitySensor(IR_Front_C11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Front_C11)
    end
    if sim.readProximitySensor(IR_Front_C12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Front_C12)
    end
    if sim.readProximitySensor(IR_Front_C13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Front_C13)
    end
    if sim.readProximitySensor(IR_Front_C14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Front_C14)
    end
    if sim.readProximitySensor(IR_Front_C15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Front_C15)
    end
    if sim.readProximitySensor(IR_Front_C16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Front_C16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[5] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Front_RR) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Front_RR)
    end
    if sim.readProximitySensor(IR_Front_RR2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Front_RR2)
    end
    if sim.readProximitySensor(IR_Front_RR3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Front_RR3)
    end
    if sim.readProximitySensor(IR_Front_RR4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Front_RR4)
    end
    if sim.readProximitySensor(IR_Front_RR5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Front_RR5)
    end
    if sim.readProximitySensor(IR_Front_RR6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Front_RR6)
    end
    if sim.readProximitySensor(IR_Front_RR7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Front_RR7)
    end
    if sim.readProximitySensor(IR_Front_RR8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Front_RR8)
    end
    if sim.readProximitySensor(IR_Front_RR9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Front_RR9)
    end
    if sim.readProximitySensor(IR_Front_RR10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Front_RR10)
    end
    if sim.readProximitySensor(IR_Front_RR11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Front_RR11)
    end
    if sim.readProximitySensor(IR_Front_RR12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Front_RR12)
    end
    if sim.readProximitySensor(IR_Front_RR13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Front_RR13)
    end
    if sim.readProximitySensor(IR_Front_RR14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Front_RR14)
    end
    if sim.readProximitySensor(IR_Front_RR15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Front_RR15)
    end
    if sim.readProximitySensor(IR_Front_RR16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Front_RR16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[6] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Back_C) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Back_C)
    end
    if sim.readProximitySensor(IR_Back_C2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Back_C2)
    end
    if sim.readProximitySensor(IR_Back_C3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Back_C3)
    end
    if sim.readProximitySensor(IR_Back_C4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Back_C4)
    end
    if sim.readProximitySensor(IR_Back_C5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Back_C5)
    end
    if sim.readProximitySensor(IR_Back_C6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Back_C6)
    end
    if sim.readProximitySensor(IR_Back_C7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Back_C7)
    end
    if sim.readProximitySensor(IR_Back_C8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Back_C8)
    end
    if sim.readProximitySensor(IR_Back_C9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Back_C9)
    end
    if sim.readProximitySensor(IR_Back_C10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Back_C10)
    end
    if sim.readProximitySensor(IR_Back_C11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Back_C11)
    end
    if sim.readProximitySensor(IR_Back_C12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Back_C12)
    end
    if sim.readProximitySensor(IR_Back_C13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Back_C13)
    end
    if sim.readProximitySensor(IR_Back_C14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Back_C14)
    end
    if sim.readProximitySensor(IR_Back_C15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Back_C15)
    end
    if sim.readProximitySensor(IR_Back_C16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Back_C16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[7] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    -- Lectura de distancias de cada sensor de proximidad y guardado en el vector "distancia"
    if sim.readProximitySensor(IR_Front_LL) == 0 then
        distancia[1] = 1000000
    else
        _res, distancia[1] = sim.readProximitySensor(IR_Front_LL)
    end
    if sim.readProximitySensor(IR_Front_LL2) == 0 then
        distancia[2] = 1000000
    else
        _res, distancia[2] = sim.readProximitySensor(IR_Front_LL2)
    end
    if sim.readProximitySensor(IR_Front_LL3) == 0 then
        distancia[3] = 1000000
    else
        _res, distancia[3] = sim.readProximitySensor(IR_Front_LL3)
    end
    if sim.readProximitySensor(IR_Front_LL4) == 0 then
        distancia[4] = 1000000
    else
        _res, distancia[4] = sim.readProximitySensor(IR_Front_LL4)
    end
    if sim.readProximitySensor(IR_Front_LL5) == 0 then
        distancia[5] = 1000000
    else
        _res, distancia[5] = sim.readProximitySensor(IR_Front_LL5)
    end
    if sim.readProximitySensor(IR_Front_LL6) == 0 then
        distancia[6] = 1000000
    else
        _res, distancia[6] = sim.readProximitySensor(IR_Front_LL6)
    end
    if sim.readProximitySensor(IR_Front_LL7) == 0 then
        distancia[7] = 1000000
    else
        _res, distancia[7] = sim.readProximitySensor(IR_Front_LL7)
    end
    if sim.readProximitySensor(IR_Front_LL8) == 0 then
        distancia[8] = 1000000
    else
        _res, distancia[8] = sim.readProximitySensor(IR_Front_LL8)
    end
    if sim.readProximitySensor(IR_Front_LL9) == 0 then
        distancia[9] = 1000000
    else
        _res, distancia[9] = sim.readProximitySensor(IR_Front_LL9)
    end
    if sim.readProximitySensor(IR_Front_LL10) == 0 then
        distancia[10] = 1000000
    else
        _res, distancia[10] = sim.readProximitySensor(IR_Front_LL10)
    end
    if sim.readProximitySensor(IR_Front_LL11) == 0 then
        distancia[11] = 1000000
    else
        _res, distancia[11] = sim.readProximitySensor(IR_Front_LL11)
    end
    if sim.readProximitySensor(IR_Front_LL12) == 0 then
        distancia[12] = 1000000
    else
        _res, distancia[12] = sim.readProximitySensor(IR_Front_LL12)
    end
    if sim.readProximitySensor(IR_Front_LL13) == 0 then
        distancia[13] = 1000000
    else
        _res, distancia[13] = sim.readProximitySensor(IR_Front_LL13)
    end
    if sim.readProximitySensor(IR_Front_LL14) == 0 then
        distancia[14] = 1000000
    else
        _res, distancia[14] = sim.readProximitySensor(IR_Front_LL14)
    end
    if sim.readProximitySensor(IR_Front_LL15) == 0 then
        distancia[15] = 1000000
    else
        _res, distancia[15] = sim.readProximitySensor(IR_Front_LL15)
    end
    if sim.readProximitySensor(IR_Front_LL16) == 0 then
        distancia[16] = 1000000
    else
        _res, distancia[16] = sim.readProximitySensor(IR_Front_LL16)
    end
    -- Calculo de la intensidad detectada por el sensor IR a partir de las distancias obtenidas de cada sensor de proximidad
    valorIR = 0
    for i = 1, 16, 1 do
        valorIR = valorIR + a * (distancia[i] ^ b)
    end
    IR[8] = valorIR --+math.random(-5,5)/100*valorIR-- Guardado del valor de la intensidad en el vector "IR" y simulacion de ruido en el sensor
    return IR, {}, {}, ""
end
