function sysCall_init()
    -- do some initialization here
    screen = sim.getObjectHandle("Screen")
end

function sysCall_cleanup()
    -- do some clean-up here
    face, img, resolution = sim.createTexture(
        "C:/Users/Juanatey/robobo/pictures/normal.jpg",
        0,
        nil,
        nil,
        nil,
        0,
        nil
    )
    sim.setShapeTexture(screen, img, sim.texturemap_cube, 1100, { 0.13, 0.13 }, nil, nil)
end

function setEmotionTo(inInts, inFloats, emotion, inBuffer)
    if emotion[1] == "happy" then
        ruta = "C:/Users/Juanatey/robobo/pictures/happy.jpg"
    elseif emotion[1] == "laughing" then
        ruta = 0
    elseif emotion[1] == "surprised" then
        ruta = 0
    elseif emotion[1] == "sad" then
        ruta = "C:/Users/Juanatey/robobo/pictures/sad.jpg"
    elseif emotion[1] == "angry" then
        ruta = "C:/Users/Juanatey/robobo/pictures/angry.jpg"
    elseif emotion[1] == "normal" then
        ruta = "C:/Users/Juanatey/robobo/pictures/normal.jpg"
    elseif emotion[1] == "sleeping" then
        ruta = "C:/Users/Juanatey/robobo/pictures/sleeping.jpg"
    elseif emotion[1] == "tired" then
        ruta = 0
    elseif emotion[1] == "afraid" then
        ruta = 0
    end
    face, img, resolution = sim.createTexture(ruta, 0, nil, nil, nil, 0, nil)
    sim.setShapeTexture(screen, img, sim.texturemap_cube, 1100, { 0.13, 0.13 }, nil, nil)
    return {}, {}, {}, ""
end
