local sim = require("sim")

function sysCall_init()
    -- do some initialization here
    print("test")
    screen = sim.getObject(".")
    defaultEmotion = ""
end

function sysCall_cleanup()
    -- do some clean-up here
    local _face, img, _resolution =
        sim.createTexture(defaultEmotion, 0, nil, nil, nil, 0, nil)
    sim.setShapeTexture(screen, img, sim.texturemap_cube, 1100, { 0.13, 0.13 }, nil, nil)
end

setDefaultEmotion = function(inIntegers, inFloats, inStrings, inBuffer)
    defaultEmotion = inStrings[1]
    return {}, {}, {}, ""
end

setEmotionTo = function(inIntegers, inFloats, inStrings, inBuffer)
    local _face, img, _resolution =
        sim.createTexture(inStrings[1], 0, nil, nil, nil, 0, nil)
    sim.setShapeTexture(screen, img, sim.texturemap_cube, 1100, { 0.13, 0.13 }, nil, nil)
    return {}, {}, {}, ""
end
