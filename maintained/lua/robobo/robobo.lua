function sysCall_init()
    status, info, serverVersion, clientVersion, clientIp = simRemoteApi.status(19999) 
    if status == -1 then
        simRemoteApi.start(19999)
    end
end
