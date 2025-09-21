-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local function load(data)
  commands.setFreeCamera()
  core_camera.setPosRot(0, data.camPos[1], data.camPos[2], data.camPos[3], data.camRot[1], data.camRot[2], data.camRot[3], data.camRot[4])
  extensions.hook('onSnapshotLoad', data)
  if core_modmanager.isReady() then
    changeMap()
  end
end

local function save(data)
  commands.setFreeCamera()
  core_camera.setPosRot(0, data.camPos[1], data.camPos[2], data.camPos[3], data.camRot[1], data.camRot[2], data.camRot[3], data.camRot[4])
  extensions.hook('onSnapshotLoad', data)
  if core_modmanager.isReady() then
    changeMap()
  end
end

-- mainentance functions below

local function createSnapshot(scheme, saveToServer)
  local data = save()
  extensions.hook('onSnapshotCreate', data)
  core_online.apiCall('s2/v4/saveSnapshot', function(request)
    if not request.responseData then
      log('E', "snapshot", "Failed to save snapshot to server")
      return
    end

    local id = request.responseData.snapshotId
    if id then
      log('I', "snapshot", "Snapshot saved, ID: " .. tostring(id))
    end
  end, {data = jsonEncode(data)})
end

local function loadSnapshot(snapshotId)
  log('I', "snapshot", "Downloading snapshot " .. tostring(snapshotId) .."...")
  core_online.apiCall('s2/v4/getSnapshot', function(request)
    if not request.responseData then
      log('E', "snapshot", "Failed to download snapshot")
      return
    end
    load(request.responseData)
  end, {snapshotId = snapshotId})
end

local function onSnapshotSchemeCommand(command, data, isStartingArg)
  if isStartingArg and M.ignoreStartupCmd then return end

  if command == "loadSnapshot" then
    loadSnapshot(data)
  end
end

local function onSerialize()
  return {
    ignoreStartupCmd = true
  }
end

local function onDeserialized(d)
  M.ignoreStartupCmd = d.ignoreStartupCmd
end

-- Public interface
M.onSnapshotSchemeCommand = onSnapshotSchemeCommand
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.createSnapshot = createSnapshot
M.loadSnapshot = loadSnapshot

return M
