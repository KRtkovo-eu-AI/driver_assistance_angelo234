-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logtag = "stitchSpline import"

-- External modules.
local splineMgr = require('editor/stitchSpline/splineMgr')
local rdp = require('editor/toolUtilities/rdp')
local fit = require('editor/toolUtilities/fitPoly')
local util = require('editor/toolUtilities/util')


-- Converts a list of ordered TSStatic objects into a stitch spline.
local function convertComponents2StitchSpline(poles, wires)
  -- Collect the positions of all the ordered objects.
  local positions, widths = {}, {}
  for _, data in ipairs(poles) do
    data.obj = Sim.upcast(data.obj)
    local pos = vec3(data.obj:getPosition())
    table.insert(positions, pos)
    table.insert(widths, 10.0) -- Use dummy widths.
  end

  -- Simplify the positions using the Ramer-Douglas-Peucker algorithm, up to some tolerance.
  local simplifiedPositions = rdp.rdp(positions, 0.5)

  -- Filter out nodes too close in XY-plane, up to some tolerance.
  local filteredPositions = util.filterClosePointsXY(simplifiedPositions, 5.0)

  -- Create a new stitch spline.
  local stitchSplines = splineMgr.getStitchSplines()
  splineMgr.addNewStitchSpline()
  local spline = stitchSplines[#stitchSplines]
  spline.name = spline.name .. " [Imported]"
  spline.poleMeshPath = poles[1].meshName
  spline.poleMeshName = poles[1].meshName:match("([^/]+)$")
  spline.wireMeshPath = wires[1].meshName
  spline.wireMeshName = wires[1].meshName:match("([^/]+)$")
  spline.nodes = filteredPositions
  spline.widths = widths
  spline.nmls = {}
  for _ = 1, #filteredPositions do
    table.insert(spline.nmls, vec3(0, 0, 1))
  end
  spline.isDirty = true

  -- Estimate the spacing between the pole components.
  local actualLength = util.calculateAverageSpacingXY(positions, 1.0) -- Calculate the average spacing between the ordered components.
  local boxLength = spline.pole_extentsL_Center
  spline.spacing = actualLength - boxLength
end

-- Undo function for converting TSStatic objects to stitch splines.
local function convertTSStatics2StitchSpline_Undo(data)
  -- Remove current splines cleanly using provided helper
  splineMgr.removeAllStitchSplines(true)

  -- Restore previous splines
  splineMgr.setStitchSplines(data.old)

  -- Recreate deleted TSStatic objects
  if data.deletedTSStatics then
    for _, entry in ipairs(data.deletedTSStatics) do
      local obj = createObject("TSStatic")
      obj:setField("shapeName", 0, entry.shapeName)
      obj:setPosition(entry.pos)
      if type(entry.rot) == "string" then
        local x, y, z, w = string.match(entry.rot, "([%-%d%.eE]+)%s+([%-%d%.eE]+)%s+([%-%d%.eE]+)%s+([%-%d%.eE]+)")
        if x and y and z and w then
          obj:setField("rotation", 0, string.format("%s %s %s %s", x, y, z, w))
        else
          log("W", logtag, "Invalid rotation format in undo. Skipping setRotation.")
        end
      end
      obj.scale = entry.scale
      if entry.name then
        obj:registerObject(entry.name)
      else
        local fallbackName = (entry.shapeName:match("([^/]+)%.dae$") or entry.shapeName:match("([^/]+)$")) or "RestoredTSStatic"
        obj:registerObject(fallbackName)
      end
      local parent = entry.groupId and scenetree.findObjectById(entry.groupId)
      if parent then
        parent:addObject(obj)
      else
        scenetree.MissionGroup:addObject(obj)
      end
    end
  end

  editor.refreshSceneTreeWindow()
end

-- Redo function for converting TSStatic objects to stitch splines.
local function convertTSStatics2StitchSpline_Redo(data)
  -- Remove current splines cleanly using provided helper
  splineMgr.removeAllStitchSplines(true)

  -- Restore new (post-conversion) splines
  splineMgr.setStitchSplines(data.new)

  -- Delete the previously restored TSStatic objects (if any still exist)
  if data.deletedTSStatics then
    for _, entry in ipairs(data.deletedTSStatics) do
      if entry.name then
        local obj = scenetree.findObject(entry.name)
        if obj and obj:getClassName() == "TSStatic" then
          obj:delete()
        end
      end
    end
  end

  -- Refresh the scene tree window after the conversion.
  editor.refreshSceneTreeWindow()
end

-- Convert the selected TSStatic objects to a stitch spline.
local function convertTSStatics2StitchSpline(collection)
  if not collection or #collection == 0 then
    log("W", logtag, "No TSStatic objects selected.")
    return
  end

  local preState = splineMgr.deepCopyStitchSplineState()
  local poleComponents, wireComponents = {}, {}

  -- Collect valid TSStatic objects from selection.
  for _, objId in ipairs(collection) do
    local obj = scenetree.findObjectById(objId)
    local isPole = string.find(obj.shapeName, "pole")
    local isWire = string.find(obj.shapeName, "wire")
    if obj and obj:getClassName() == "TSStatic" and (isPole or isWire) then
      local shape = obj.shapeName
      if shape and shape ~= "" then
        if isPole then
          table.insert(poleComponents, { obj = obj, meshName = shape })
        elseif isWire then
          table.insert(wireComponents, { obj = obj, meshName = shape })
        end
      end
    end
  end

  if #poleComponents < 2 or #wireComponents < 1 then
    log("W", logtag, "Need at least two valid TSStatic pole objects and one valid TSStatic wire object to form a stitch spline.")
    return
  end

  -- Fit the pole components to a polyline.
  local ordered = fit.fitPoly(poleComponents)

  -- Backup the TSStatic objects.
  local tsStaticBackup = {}
  for _, comp in ipairs(ordered) do
    local obj = comp.obj
    if obj then
      local group = obj:getGroup()
      table.insert(tsStaticBackup, {
        id = obj:getID(),
        name = obj:getName(),
        shapeName = obj.shapeName,
        pos = obj:getPosition(),
        rot = obj:getField("rotation", 0),
        scale = obj.scale,
        groupId = group and group:getID() or nil
      })
    end
  end
  for _, comp in ipairs(wireComponents) do -- Include wire components in the backup, so we can delete them later.
    local obj = comp.obj
    if obj then
      local group = obj:getGroup()
      table.insert(tsStaticBackup, {
        id = obj:getID(),
        name = obj:getName(),
        shapeName = obj.shapeName,
        pos = obj:getPosition(),
        rot = obj:getField("rotation", 0),
        scale = obj.scale,
        groupId = group and group:getID() or nil
      })
    end
  end

  -- Convert to stitch spline.
  convertComponents2StitchSpline(ordered, wireComponents)

  -- Delete used TSStatic objects, and clean up parent groups if now empty.
  local cleanedGroups = {}
  for _, comp in ipairs(ordered) do
    local obj = comp.obj
    if obj and Sim.upcast(obj) then
      local group = obj:getGroup()
      obj:delete()

      if group and Sim.upcast(group) and not cleanedGroups[group:getID()] then
        local onlyTSStatics = true
        for i = 0, group:size() - 1 do
          local sibling = group:at(i)
          if sibling and Sim.upcast(sibling) and sibling:getClassName() ~= "TSStatic" then
            onlyTSStatics = false
            break
          end
        end

        if onlyTSStatics and group:size() == 0 then
          group:delete()
        end

        cleanedGroups[group:getID()] = true
      end
    end
  end

  -- Delete the wire components.
  for _, comp in ipairs(wireComponents) do
    comp.obj:delete()
  end

  local postState = splineMgr.deepCopyStitchSplineState()
  editor.history:commitAction(
    "Convert TSStatic to Stitch Spline",
    { old = preState, new = postState, deletedTSStatics = tsStaticBackup },
    convertTSStatics2StitchSpline_Undo,
    convertTSStatics2StitchSpline_Redo
  )

  -- Refresh the scene tree window after the conversion.
  editor.refreshSceneTreeWindow()

  -- Rebuild the collision mesh (after the old object removal and before the new objects are generated on the first update)
  be:reloadCollision()
end

-- Import a stitch spline from the given selection polygon.
local function importFromPolygon(polygon)
  local meshesInPolygon = {}
  for _, meshName in pairs(scenetree.findClassObjects("TSStatic")) do
    local obj = scenetree.findObject(meshName)
    if obj then
      local pos = obj:getPosition()
      if pos:inPolygon(polygon) then
        table.insert(meshesInPolygon, obj:getID())
      end
    end
  end
  convertTSStatics2StitchSpline(meshesInPolygon)
end


-- Public interface.
M.convertTSStatics2StitchSpline =                       convertTSStatics2StitchSpline
M.importFromPolygon =                                   importFromPolygon

return M