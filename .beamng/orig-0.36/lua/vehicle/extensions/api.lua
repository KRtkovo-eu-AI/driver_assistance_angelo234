-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local httpClient = require("socket.http")
local httpJsonServer = require('utils/httpJsonServer')


local function jsonRequest(uri, data)
    --[[
    -- GET
    local respbody = {}
    local body, code, headers, status = httpClient.request {
        method = 'GET',
        url = uri,
        sink = ltn12.sink.table(respbody)
    }

    --]]
    local respbody = {}
    local reqbody = "data=" .. jsonEncode(data)
    local body, code, headers, status = httpClient.request {
        method = 'POST',
        url = uri,
        source = ltn12.source.string(reqbody),
        headers = {
            ["Accept"] = "*/*",
            --["Accept-Encoding"] = "gzip, deflate",
            --["Accept-Language"] = "en-us",
            ["Content-Type"] = "application/x-www-form-urlencoded",
            ["content-length"] = string.len(reqbody)
        },
        sink = ltn12.sink.table(respbody)
    }
    if code ~= 200 then
        return {ok = false, error = code}
    end

    --print('body:' .. tostring(body))
    --print('code:' .. tostring(code))
    --print('headers:' .. dumps(headers))
    --print('status:' .. tostring(status))
    return jsonDecode(table.concat(respbody), 'json request response')
end

local function prepareEditorData()
    --local data_ref = {nodes = v.data.nodes, beams = v.data.beams}
    --local whitelist = {pos=1, name=1, id1=1, id2=1, beamType=1, cid=1}
    -- for nodes, beams
    --[[
    local data = {}
    for vk,vs in pairs(data_ref) do
        data[vk] = {}
        -- for nodeid, beamnid
        for _,vv in pairs(vs) do
            data[vk][vv.cid] = {}
            for vk,vi in pairs(vv) do
                if whitelist[vk] then
                    data[vk][vv.cid][vk] = vi
                end
            end
        end
    end]]--
    local editorData = {}
    local nodes = {}
    local beams = {}
    local tris = {}
    for k, s in pairs(v.data.beams) do
        if not s.wheelID then
            beams[k] = {s.cid, {s.id1,s.id2}, s.beamSpring, s.beamDamp}
        end
    end
    for k, s in pairs(v.data.nodes) do
        if not s.wheelTreadBeamDeform then
            nodes[k] = {s.cid, {s.pos.x, s.pos.y, s.pos.z}, s.nodeWeight}
        end
    end
    for k, s in pairs(v.data.triangles) do
        if s.cid then
            tris[k] = {{s.id1, s.id2, s.id3}}
        end
    end
    editorData = {nodes, beams, tris}
    return editorData
end

local function handleServerRequest(request)
    --print("got request:")
    --dump(request)
    if not request.uri then return end -- returns 500

    if request.uri.path == 'v1/ping' then
        return {ok = true}

    elseif request.uri.path == 'v1/getData' then
        local data = prepareEditorData()
        --dump(data)
        return data
    end
    -- returning nil results in 500 error
end

local inited = false

local function onExtensionLoaded()

    local port = 23512
    local bindHost = 'localhost'
    if not inited then
        log('E', "default.init", "INIT WEBSERVER TO PORT "..port)
        httpJsonServer.init(bindHost, port, handleServerRequest)
        inited = true
    end


    -- no self test :( - it would block
    --local res = jsonRequest('http://localhost:23716/v1/register', {server = bindHost, port = port})
    --dump(res)
end


local function onVehicleLoaded()
    --dump(prepareEditorData())
end

local function updateGFX()
    httpJsonServer.update()
end


-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX
M.onVehicleLoaded = onVehicleLoaded

return M
