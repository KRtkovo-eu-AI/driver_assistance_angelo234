-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local logTag = ''

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local Recce = require('/lua/ge/extensions/gameplay/rally/recce')

local C = {}
C.windowDescription = 'Recce'

function C:init(rallyEditor)
  self.path = nil
  self.rallyEditor = rallyEditor
  self.recce = nil
  -- use this to draw timestamp labels on driveline points.
  -- Click on a point to copy TS value to clipboard for easy manual driveline editing.
  self.drawLabels = false
  self.originalDrivelineSize = nil
end

function C:setPath(path)
  self.path = path
end

-- called by RallyEditor when this tab is selected.
function C:selected()
  if not self.path then return end
  self:refresh()
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

-- called by RallyEditor when this tab is unselected.
function C:unselect()
  if not self.path then return end
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

function C:formatDistance(dist)
  local unit = 'm'
  if dist > 950 then
    dist = dist / 1000.0
    unit = 'km'
  end

  local dist_str = string.format("%.3f"..unit, dist)
  return dist_str
end

function C:downsample()
  if not self.recce then return end
  self.recce.driveline = self.recce.driveline:downsample(self.straightnessThreshold)
end

function C:drawSectionV3()
  im.HeaderText("Recce Recording")
  if im.Button("Refresh") then
    self:refresh()
  end

  if im.Checkbox("Show Labels (click to copy)", im.BoolPtr(self.drawLabels)) then
    self.drawLabels = not self.drawLabels
  end

  if not (self.recce and self.recce:drivelineAndCutsLoaded()) then
    im.Text('To Import Pacenotes, make sure there is a recce recording.')
    return
  end

  if self.recce.driveline then
    local dist = self.recce.driveline:length()

    im.Text('driveline: '..tostring(#self.recce.driveline.points)..' points, '..self:formatDistance(dist))
    if #self.recce.cuts > 0 then
      im.Text('cuts: '..tostring(#self.recce.cuts)..' (the little green cars)')
    else
      im.Text('cuts: '..tostring(#self.recce.cuts))
    end
  else
    im.Text('Recorded driveline was not found.')
    im.Text(
      'A driveline is required to make pacenotes. '..
      'Using a driveline makes creating pacenotes much easier. '..
      'To record a driveline, use the Recce UI app in freeroam.'
    )
  end

  if self.recce.cuts then
    if im.Button("Import") then
      self:import()
    end
    im.Text('Import will create a new pacenote for each of the cuts.')
  end

  im.Separator()

  if im.Button("Downsample") then
    self:downsample()
  end
  im.SameLine()
  im.Text("Original size: "..self.originalDrivelineSize)
  im.SameLine()
  im.Text("Downsampled size: "..#self.recce.driveline.points)
  im.SameLine()
  local pointsRemoved = self.originalDrivelineSize - #self.recce.driveline.points
  local percentageRemoved = 0
  if self.originalDrivelineSize > 0 then
    percentageRemoved = (pointsRemoved / self.originalDrivelineSize) * 100
  end
  im.Text(string.format("Downsampled by: %d points (%.1f%%%%)", pointsRemoved, percentageRemoved))

  im.Text("Straightness Threshold")
  im.SameLine()
  im.SetNextItemWidth(120)
  if not self.straightnessThreshold then
    self.straightnessThreshold = 1.0 -- Default value (closer to 1 means straighter)
  end
  local ptr = im.FloatPtr(self.straightnessThreshold)
  if im.SliderFloat("##StraightnessThreshold", ptr, 0.980, 1.0, "%.6f") then
    -- Value changed by user
    self.straightnessThreshold = ptr[0]
  end
  im.SameLine()
  im.TextDisabled("(?)")
  if im.IsItemHovered() then
    im.BeginTooltip()
    im.Text("Controls how aggressively to downsample the driveline.")
    im.Text("Higher values (closer to 1) keep more points.")
    im.Text("Lower values remove more points on straight sections.")
    im.EndTooltip()
  end
end

function C:refresh()
  self.recce = Recce(self.path:getMissionDir())
  if not self.recce:loadDrivelineAndCuts() then
    log('E', logTag, 'failed to load recce driveline and cuts for refresh')
    return
  end
  self.originalDrivelineSize = #self.recce.driveline.points
end

function C:import()
  local pacenotes = self.recce:createPacenotesData(self.path)
  if pacenotes then
    self.path:appendPacenotes(pacenotes)
    -- self.path:normalizeAllFreeformNotes()
    self.rallyEditor.showPacenotesTab()
  end
end

function C:draw(mouseInfo)
  if not self.path then return end
  self:drawSectionV3()
end

function C:drawDebugEntrypoint(mouseInfo)
  if not self.recce then return end
  self.recce:drawDebugRecce(self.drawLabels, mouseInfo)
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

