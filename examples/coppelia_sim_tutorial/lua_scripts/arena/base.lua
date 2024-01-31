sim = require("sim")

local sensor
local food
local distance

function sysCall_init()
    sensor = sim.getObject("./Base_Proximity_sensor")
    food = sim.getObject("/Food")
    distance = -1.0
end

function sysCall_actuation()
    detected, dist, points, obj, n = sim.checkProximitySensor(sensor, food)
    if detected then
        distance = dist
    else
        distance = -1.0
    end
end

getFoodDistance = function(inIntegers, inFloats, inStrings, inBuffer)
    return {}, { distance }, {}, ""
end
