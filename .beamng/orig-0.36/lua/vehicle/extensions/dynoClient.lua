-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local dynoObjectID = nil
local textureName = nil

local function sendData(textureName, jsFunc, data, objId)
    local js = jsonEncode(data):gsub('%"', '%\\\'') -- replace " with \'
    local l = "getObjectByID("..objId.."):queueJSUITexture('"..textureName.."', '"..jsFunc.."(" .. js .. ");')"
    obj:queueGameEngineLua(l)
end

local function initDyno(_dynoObjectID, _textureName)
    if not _dynoObjectID then return end
    dynoObjectID = _dynoObjectID
    textureName = _textureName

    local frontAxis = 0.0
    local rearAxis = 0.0
    for k,wd in pairs(v.data.wheels) do
        frontAxis =  math.min( frontAxis, v.data.nodes[wd.node1].pos.y  )
        rearAxis =  math.max( rearAxis, v.data.nodes[wd.node1].pos.y  )
    end

    local data = {
        vehicle = v.data.vehicleDirectory,
        id = obj:getId(),
        -- TODO: add more init data in here
    }
    sendData(textureName, 'client_init', data, dynoObjectID)
end

local function updateGFX(dt)
    if not dynoObjectID then return end

    local gear = electrics.values.gear_M
    local rpm = drivetrain.rpm
    local gearRatio = 1
    local finalRatio = 1
    if v.data.engine then
        if v.data.engine.gears then
            gearRatio = v.data.engine.gears[gear]
            if gearRatio == 0 then gearRatio = 1 end
        end
        if v.data.engine.differential then
            finalRatio = v.data.engine.differential
            if finalRatio == 0 then finalRatio = 1 end
        end
    end
    local data = {
        -- you can put more stuff here that is required during runtime
        rpm = rpm,
        gear = gear,
        gearRatio = gearRatio,
        finalRatio = finalRatio,
    }
    --dump(data)
    sendData(textureName, 'client_data', data, dynoObjectID)
end

-- public interface
M.initDyno = initDyno
M.updateGFX = updateGFX

return M
