local sim = require("sim")

function sysCall_init()
    local status, _info, _serverVersion, _clientVersion, _clientIp =
        simRemoteApi.status(19999)
    if status == -1 then
        print([[
            Warning: you did not start CoppeliaSim in the way required to connect to the server.
            Please start with "-gREMOTEAPISERVERSERVICE_19999_FALSE_TRUE",
            using ./scripts/start_coppelia_sim
        ]])
    end
end
