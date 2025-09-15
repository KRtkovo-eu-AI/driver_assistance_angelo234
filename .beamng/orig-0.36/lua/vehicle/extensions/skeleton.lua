-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- this module is always loaded and then unloaded if not required.
-- it draws the basic skeleton of everything in case no flexbody is found
-- this is supposed to help people get quickstarted in working with jbeam

local M = {}

local function onExtensionLoaded()
  if (v.data.beams == nil or v.data.nodes == nil)
     or (not v.data.information.showSkeleton)
     or (v.data.flexbodies ~= nil and not tableIsEmpty(v.data.flexbodies)) then

    -- unload module
    return false
  end

  -- load module
  return true
end

local function onDebugDraw(focusPos)
  -- this is disabled once debug mode is enabled, so it does not conflict
  if bdebug.isEnabled() then
    return
  end

  for _, beam in pairs(v.data.beams) do
    obj.debugDrawProxy:drawBeam3d(beam.cid, 0.01, color(44, 71, 112, 230))
  end

  for _, node in pairs(v.data.nodes) do
    obj.debugDrawProxy:drawNodeSphere(node.cid, 0.03, color(170, 57, 57, 230))
  end

  obj.debugDrawProxy:drawColTris(0, color(0, 0, 0, 150), color(0, 100, 0, 50), color(100, 0, 0, 50), 1, color(0, 0, 255, 255))
end

M.onDebugDraw = onDebugDraw
M.onExtensionLoaded = onExtensionLoaded

return M
