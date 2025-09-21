-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is an example on how to use the rawLidar interface. The specific example uses
-- a shared library for the data, the source code of which is located in `/tech/Raw_LiDAR`.
-- To run this example, you will need to compile the library and call
-- `extensions.tech_sensors.createRawLidar(
--    be:getPlayerVehicleID(0), 'tech/rawLidar', {dllFilename = 'C:/absolute/path/to/rawLidar.dll'})`

-- User-defined arguments.
local defaultDllFilename = 'C:/rawLidar.dll'

-- Private state.
local userDLL = nil

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

  -- ffi.load only works with disabled sandbox
  if Engine.Sandbox.Lua.isEnabled() then
    log('E', logTag, 'This feature can only run when the Lua security sandbox is disabled. ' ..
      'You will have to restart BeamNG with the \'-disable-sandbox\' argument.')
    return nil
  end

  local dllFilename
  if not args or not args.dllFilename then
    dllFilename = defaultDllFilename
  else
    dllFilename = args.dllFilename
  end

  if not dllFilename then
    log('E', logTag, 'Shared library name not provided!')
    return nil
  end

  -- Handshake the .dll and store a reference to it in this module.
  userDLL = ffi.load(dllFilename)
  if not userDLL then
    log('E', logTag, 'ERROR - Could not load user .dll at the given path.')
    return nil
  end
  if not _G['__rawLidar_cdef'] then
  ffi.cdef
  [[
    struct initData {
      double posX;
      double posY;
      double posZ;
      float dirX;
      float dirY;
      float dirZ;
      float upX;
      float upY;
      float upZ;
      int resX;
      int resY;
      float fovY;
      float pNear;
      float pFar;
      float overlap;
    };
    struct initData bng_rawLidar_onInit();
    void bng_rawLidar_onUpdate(float, float*, float*, float*, float*, char*, char*, char*, char*);
    void bng_rawLidar_onRemove();
  ]]
  rawset(_G, '__rawLidar_cdef', true)
  end

  local initData = userDLL.bng_rawLidar_onInit()                                                    -- Call the user's initialisation function in the .dll and get the setup data.

  return initData                                                                                   -- Send the setup data back to the system.
end

-- This function is called once when the Raw LiDAR sensor is removed.
local function onRemove()
  userDLL.bng_rawLidar_onRemove()                                                                   -- Call the remove callback in the .dll.
end

-- This function is called in every frame, and contains the latest Raw LiDAR sensor readings.
local function onUpdate(dt, depth1, depth2, depth3, depth4, annot1, annot2, annot3, annot4)
    userDLL.bng_rawLidar_onUpdate(                                                                    -- Send the data to the user's .dll, in the per-frame update callback.
    dt,
    depth1,
    depth2,
    depth3,
    depth4,
    annot1,
    annot2,
    annot3,
    annot4)
end

-- Public interface.

M.onInit =                                              onInit
M.onRemove =                                            onRemove
M.onUpdate =                                            onUpdate

return M