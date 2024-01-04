local sim = require("sim")

function sysCall_init()
    -- do some initialization here
    Back_L = sim.getObject("../Back_L")
    Back_R = sim.getObject("../Back_R")
    Front_C = sim.getObject("../Front_C")
    Front_L = sim.getObject("../Front_L")
    Front_LL = sim.getObject("../Front_LL")
    Front_R = sim.getObject("../Front_R")
    Front_RR = sim.getObject("../Front_RR")
end

function sysCall_cleanup()
    -- do some clean-up here
    sim.setShapeColor(Back_L, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
    sim.setShapeColor(Back_R, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
    sim.setShapeColor(Front_C, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
    sim.setShapeColor(Front_L, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
    sim.setShapeColor(Front_LL, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
    sim.setShapeColor(Front_R, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
    sim.setShapeColor(Front_RR, nil, sim.colorcomponent_ambient_diffuse, { 1, 1, 1 })
end

setLEDColor = function(inIntegers, inFloats, parametros, inBuffer) --parametros=[led,color]
    led = parametros[1]
    color = parametros[2]
    --comprobamos el color
    if color == "off" then
        rgb = { 1, 1, 1 }
    elseif color == "white" then
        rgb = { 1, 1, 1 }
    elseif color == "red" then
        rgb = { 1, 0, 0 }
    elseif color == "blue" then
        rgb = { 0, 0, 1 }
    elseif color == "cyan" then
        rgb = { 0, 1, 1 }
    elseif color == "magenta" then
        rgb = { 1, 0, 1 }
    elseif color == "yellow" then
        rgb = { 1, 1, 0 }
    elseif color == "green" then
        rgb = { 0, 1, 0 }
    elseif color == "orange" then
        rgb = { 1, 0.5, 0 }
    end
    --encendemos los led correspondientes
    if led == "Back_L" then
        sim.setShapeColor(Back_L, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "Back_R" then
        sim.setShapeColor(Back_R, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "Front_C" then
        sim.setShapeColor(Front_C, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "Front_L" then
        sim.setShapeColor(Front_L, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "Front_LL" then
        sim.setShapeColor(Front_LL, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "Front_R" then
        sim.setShapeColor(Front_R, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "Front_RR" then
        sim.setShapeColor(Front_RR, nil, sim.colorcomponent_ambient_diffuse, rgb)
    elseif led == "all" then
        sim.setShapeColor(Back_L, nil, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setShapeColor(Back_R, nil, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setShapeColor(Front_C, nil, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setShapeColor(Front_L, nil, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setShapeColor(Front_LL, nil, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setShapeColor(Front_R, nil, sim.colorcomponent_ambient_diffuse, rgb)
        sim.setShapeColor(Front_RR, nil, sim.colorcomponent_ambient_diffuse, rgb)
    end
    return {}, {}, {}, ""
end
