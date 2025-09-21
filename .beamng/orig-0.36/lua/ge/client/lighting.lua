-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.reloadLightingSystems = function ()
  local adapterCount = GFXInit.getAdapterCount()
  -- log('I', 'lightManager', 'initLightingSystems called.....')
  -- log('I', 'lightManager', 'adapterCount = '..tostring(adapterCount))

  if adapterCount == 1 and GFXInit.getAdapterName(0) == "GFX Null Device" then
    -- log('D','lightManager',"Null graphics device detected, skipping Lighting Systems initialization.");
    -- CANNOT RETURN HERE, WOULD CRASH LATER
  end

  -- log('I', 'lightManager', '--------- Initializing Lighting Systems ---------')

  -- First exec the scripts for the different light managers
  -- in the lighting folder.

  -- log('I', 'lightManager', 'Finding all the system cs files')
  -- local files = FS:findFiles('/core/scripts/client/lighting/', 'init.cs', 1, true, false)
  -- dump(files)
  -- for _,filepath in ipairs(files) do
  --   TorqueScriptLua.exec(filepath)
  -- end

  -- log('I', 'lightManager', 'Finding all the system lua files')
  local files = FS:findFiles('lua/ge/client/lighting/', 'init.lua', 1, true, false)
  -- dump(files)
  for _,filepath in ipairs(files) do
    local dir, filename, ext = path.splitWithoutExt(filepath)
    -- log('I', 'lightManager', '     loading manager   '..tostring(dir..filename))
    require(dir..filename)
  end

  -- log('I', 'lightManager', 'Finished with system files')

  -- Try the perfered one first.
  local succeeded = setLightManager(getConsoleVariable('$pref::lightManager'))
  if not succeeded then
    log('E', 'lightManager', 'Failed to init default system:'..getConsoleVariable('$pref::lightManager'))

    -- The perfered one fell thru... so go thru the default
    -- light managers until we find one that works.
    local defaultLightManagerNames = settings.getValue('defaultLightManagerNames')
    local lightManagersNames = split(defaultLightManagerNames, '\n')
    for _, managerName in ipairs(lightManagersNames) do
      succeeded = setLightManager(managerName)
      if succeeded then
        break
      end
    end
  end

  -- Did we completely fail to initialize a light manager?
  if not succeeded then
    -- If we completely failed to initialize a light
    -- manager then the 3d scene cannot be rendered.
    quitWithErrorMessage( "Failed to set a light manager!" );
  end
end

M.initLightingSystems = function ()
  local adapterCount = GFXInit.getAdapterCount()
  -- log('I', 'lightManager', 'initLightingSystems called.....')
  -- log('I', 'lightManager', 'adapterCount = '..tostring(adapterCount))

  if adapterCount == 1 and GFXInit.getAdapterName(0) == "GFX Null Device" then
    -- log('D','lightManager',"Null graphics device detected, skipping Lighting Systems initialization.");
    -- CANNOT RETURN HERE, WOULD CRASH LATER
  end

  -- log('I', 'lightManager', '--------- Initializing Lighting Systems ---------')

  -- First exec the scripts for the different light managers
  -- in the lighting folder.

  -- log('I', 'lightManager', 'Finding all the system cs files')
  -- local files = FS:findFiles('/core/scripts/client/lighting/', 'init.cs', 1, true, false)
  -- dump(files)
  -- for _,filepath in ipairs(files) do
  --   TorqueScriptLua.exec(filepath)
  -- end

  -- log('I', 'lightManager', 'Finding all the system lua files')
  local files = FS:findFiles('lua/ge/client/lighting/', 'init.lua', 1, true, false)
  -- log('I', 'lightManager', '      files = '..dumps(files))
  for _,filepath in ipairs(files) do
    local dir, filename, ext = path.splitWithoutExt(filepath)
    -- log('I', 'lightManager', '     loading manager   '..tostring(dir..filename))
    require(dir..filename)
  end

  -- log('I', 'lightManager', 'Finished with system files')

  -- Try the perfered one first.
  local succeeded = setLightManager(getConsoleVariable('$pref::lightManager'))
  if not succeeded then
    log('E', 'lightManager', 'Failed to init default system:'..getConsoleVariable('$pref::lightManager'))

    -- The perfered one fell thru... so go thru the default
    -- light managers until we find one that works.
    local defaultLightManagerNames = settings.getValue('defaultLightManagerNames')
    local lightManagersNames = split(defaultLightManagerNames, '\n')
    for _, managerName in ipairs(lightManagersNames) do
      succeeded = setLightManager(managerName)
      if succeeded then
        break
      end
    end
  end

  -- Did we completely fail to initialize a light manager?
  if not succeeded then
    -- If we completely failed to initialize a light
    -- manager then the 3d scene cannot be rendered.
    quitWithErrorMessage( "Failed to set a light manager!" );
  end
end

return M
