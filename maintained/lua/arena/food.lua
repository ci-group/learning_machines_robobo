string.startswith = function(self, str)
    return self:find("^" .. str) ~= nil
end

local food_collected
local already_eaten

function sysCall_init()
    -- do some initialization here:

    -- Make sure you read the section on "Accessing general-type objects programmatically"
    -- For instance, if you wish to retrieve the handle of a scene object, use following instruction:
    --
    -- handle=sim.getObjectHandle('sceneObjectName')
    --
    -- Above instruction retrieves the handle of 'sceneObjectName' if this script's name has no '#' in it
    --
    -- If this script's name contains a '#' (e.g. 'someName#4'), then above instruction retrieves the handle of object 'sceneObjectName#4'
    -- This mechanism of handle retrieval is very convenient, since you don't need to adjust any code when a model is duplicated!
    -- So if the script's name (or rather the name of the object associated with this script) is:
    --
    -- 'someName', then the handle of 'sceneObjectName' is retrieved
    -- 'someName#0', then the handle of 'sceneObjectName#0' is retrieved
    -- 'someName#1', then the handle of 'sceneObjectName#1' is retrieved
    -- ...
    --
    -- If you always want to retrieve the same object's handle, no matter what, specify its full name, including a '#':
    --
    -- handle=sim.getObjectHandle('sceneObjectName#') always retrieves the handle of object 'sceneObjectName'
    -- handle=sim.getObjectHandle('sceneObjectName#0') always retrieves the handle of object 'sceneObjectName#0'
    -- handle=sim.getObjectHandle('sceneObjectName#1') always retrieves the handle of object 'sceneObjectName#1'
    -- ...
    --
    -- Refer also to sim.getCollisionhandle, sim.getDistanceHandle, sim.getIkGroupHandle, etc.
    print("Scene initialized")
    food_collected = 0
    already_eaten = {}
end

function sysCall_actuation()
    -- put your actuation code here
    --
    -- For example:
    --
    -- local position=sim.getObjectPosition(handle,-1)
    -- position[1]=position[1]+0.001
    -- sim.setObjectPosition(handle,-1,position)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

function collect_food(handle)
    -- This may be called multiple times before the object is actually moved
    -- up. The bag checks to eat it, if it's not already eaten. It's not a rabbit
    if not already_eaten[handle] then
        --print(handle)
        already_eaten[handle] = true
        pos = sim.getObjectPosition(handle, -1)
        pos[3] = pos[3] + 1
        sim.setObjectPosition(handle, -1, pos)
        food_collected = food_collected + 1
        print("Collected food " .. food_collected)
    end
end

function sysCall_contactCallback(inData)
    h1 = sim.getObjectName(inData.handle1)
    --h2 = sim.getObjectName(inData.handle2)
    if h1:startswith("Food") then
        --print( h1 .. " -> " .. h2)
        collect_food(inData.handle1)
    end
    --if h2:startswith("Food") then
    --    print( h1 .. " <- " .. h2)
    --end
    return outData
end

function remote_get_collected_food(inInts, inFloats, inStrings, inBuffer)
    -- inInts, inFloats and inStrings are tables
    -- inBuffer is a string

    -- Perform any type of operation here.

    -- Always return 3 tables and a string, e.g.:
    return { food_collected }, {}, {}, ""
end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_dynCallback(inData)
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end
--]]
