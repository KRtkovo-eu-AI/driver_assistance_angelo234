-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is an example on how to use the rawLidar interface. The specific example is empty and you can fill in the
-- implementations of the Lua functions yourself.
-- To run this example, you will need to call
-- `extensions.tech_sensors.createRawLidar(be:getPlayerVehicleID(0), 'tech/rawLidarEmpty')`

local M = {}

local logTag = 'rawLidar'

-- User callbacks.

-- This function is called once when the Raw LiDAR sensor is first created.
local function onInit(args)
  -- raw lidar only works on BeamNG.tech
  if not ResearchVerifier.isTechLicenseVerified() then
    log('E', logTag, 'This feature requires a BeamNG.tech license.')
    return nil
  end

  -- Your implementation goes here.
  return {
    posX=0.0, posY=0.0, posZ=0.3,                               -- The sensor position, relative to the center of the vehicle: posX, posY, posZ.
    dirX=0.0, dirY=-1.0, dirZ=0.0,                              -- The sensor 'forward' direction vector: dirX, dirY, dirZ.
    upX=0.0, upY=0.0, upZ=1.0,                                  -- The sensor 'up' direction vector: upX, upY, upZ.
    resX=1920, resY=1080,                                       -- The horizontal and vertical resolution of the depth/annotation images: resX, resY.
    fovY=1.22173,                                               -- The vertical field of view, in radians: fovY.
    pNear=0.1, pFar=1000.0,                                     -- The near and far planes: pNear, pFar.
    overlap=0.087                                               -- The signed overlap of horizontal FOV, for each quadrant camera, in radians. eg 0.0f = no overlap.
  }
end

-- This function is called once when the Raw LiDAR sensor is removed.
local function onRemove()
  -- Your implementation goes here.
end

-- This function is called in every frame, and contains the latest Raw LiDAR sensor readings.
local function onUpdate(dt, depth1, depth2, depth3, depth4, annot1, annot2, annot3, annot4)
  -- Your implementation goes here.
end

-- Public interface.

M.onInit =                                              onInit
M.onRemove =                                            onRemove
M.onUpdate =                                            onUpdate

return M