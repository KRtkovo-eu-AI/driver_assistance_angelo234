-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local im  = ui_imgui
local C = {}
local level = {'check','warning','error'}
local icons  = {'check','warning','error'}
local infoColors = {
  warning = im.ImVec4(1, 1, 0, 1.0),
  error = im.ImVec4(1, 0, 0, 1.0)
}

function C:init(missionEditor)
  self.missionEditor = missionEditor
  self.issues = {list = {}}
end

function C:setMission(mission)
  self.mission = mission
end

function C:draw()
  if not self.mission._issueList then return end
  im.Columns(2)
  im.SetColumnWidth(0,150)

  im.Text("Issues")
  im.NextColumn()
  if self.mission._issueList.importantCount == 0 then
    im.Text("No Issues!")
  end

  for _, issue in ipairs(self.mission._issueList) do
    im.BulletText(issue.label)
  end
  im.Columns(1)

end

local severityPriority = {unknown = -1, minor=1, warning = 10, error = 50, critical = 100}

local function sortByIdx(a,b) return a.idx < b.idx end
local function sortBySeverity(a,b)
  if severityPriority[a.severity] == severityPriority[b.severity] then
    return sortByIdx(a,b)
  else
    return severityPriority[a.severity] < severityPriority[b.severity]
  end
end
local function sortByMission(a,b)
  if a.missionId == b.missionId then
    return sortByIdx(a,b)
  else
    return a.missionId < b.missionId
  end
end
local function sortByMissionType(a,b)
  if a.missionType == b.missionType then
    return sortByIdx(a,b)
  else
    return a.missionType < b.missionType
  end
end
local function sortByLevel(a,b)
  if a.level == b.level then
    return sortByIdx(a,b)
  else
    return a.level < b.level
  end
end
local function sortByLabel(a,b)
  if a.label == b.label then
    return sortByIdx(a,b)
  else
    return a.label < b.label
  end
end
local function sortByAvailability(a,b)
  if a.availability == b.availability then
    return sortByIdx(a,b)
  else
    return a.availability < b.availability
  end
end

local function getSortingFunction(columnIdx)
  if columnIdx == 0 then return sortByIdx end
  if columnIdx == 1 then return sortBySeverity end
  if columnIdx == 2 then return sortByMission end
  if columnIdx == 3 then return sortByMissionType end
  if columnIdx == 4 then return sortByLevel end
  if columnIdx == 5 then return sortByAvailability end
  if columnIdx == 6 then return sortByLabel end
  return sortByIdx
end

local severityColors = {
  error = im.ImVec4(1,0.25, 0.3, 0.95),
  warning = im.ImVec4(1,0.85, 0.15, 0.95),
  minor = im.ImVec4(0.15,0.85, 1, 0.95),
}
local function getSeverityColor(type)
  return severityColors[type] or im.ImVec4(1, 1, 1, 0.85)
end

local tableFlags = bit.bor(im.TableFlags_Hideable, im.TableFlags_ScrollY, im.TableFlags_Resizable, im.TableFlags_RowBg, im.TableFlags_Reorderable, im.TableFlags_Sortable, im.TableFlags_Borders)
function C:drawIssuesWindow()
  if editor.beginWindow('mission_issues', "Mission Issues Overview",  im.WindowFlags_MenuBar) then
    if im.BeginMenuBar() then
      if im.BeginMenu("Issues") then
        if im.MenuItem1("Attempt to fix all Missiontype issues for all missions") then
          for _, mission in ipairs(gameplay_missions_missions.getFilesData()) do
            self.missionEditor.setMissionById(mission.id, true)
            self.missionEditor.getCurrentEditorHelperWhenActive():checkContainer(mission, true)
          end

        end
        if im.MenuItem1("Attempt to remove all additional data from missionTypeData") then
          for _, mission in ipairs(gameplay_missions_missions.getFilesData()) do
            self.missionEditor.setMissionById(mission.id, true)
            self.missionEditor.getCurrentEditorHelperWhenActive():checkContainer(mission, true, {})
          end

        end

        im.EndMenu()
      end

      im.EndMenuBar()
    end

    if im.BeginTable('', 7, tableFlags) then
      im.TableSetupScrollFreeze(0,1)
      im.TableSetupColumn("#",nil,4)
      im.TableSetupColumn("Type",nil,5) -- severity
      im.TableSetupColumn("Mission",nil,20)
      im.TableSetupColumn("MissionType",im.TableColumnFlags_DefaultHide,5)
      im.TableSetupColumn("Level",im.TableColumnFlags_DefaultHide,5)
      im.TableSetupColumn("Availability",im.TableColumnFlags_DefaultHide,5)
      im.TableSetupColumn("Label", nil,60)
      im.TableHeadersRow()
      im.TableNextColumn()
      if im.TableGetSortSpecs().SpecsDirty then

        table.sort(self.issues.list, getSortingFunction(im.TableGetSortSpecs().Specs.ColumnIndex))
        if im.TableGetSortSpecs().Specs.SortDirection == 1 then
          arrayReverse(self.issues.list)
        end
        im.TableSetSortSpecsDirty(false)
      end

      for _, issue in ipairs(self.issues.list or {}) do
        im.Text(issue.idx.."")
        im.TableNextColumn()
        im.TextColored(getSeverityColor(issue.severity), issue.severity)
        im.TableNextColumn()
        local name = issue.missionId
        if editor.getPreference('missionEditor.general.shortIds') then
          local p, fn, _ = path.split(name)
          name = fn
        end
        if im.Selectable1(name..'##'..name..'-'..issue.idx) then
          self.missionEditor.setMissionById(issue.missionId)
        end
        im.TableNextColumn()
        im.Text(issue.missionType)
        im.TableNextColumn()
        im.Text(issue.level)
        im.TableNextColumn()
        im.Text(issue.availability)
        im.TableNextColumn()
        im.Text(issue.label)
        im.TableNextColumn()
      end
      im.EndTable()
    end

    editor.endWindow()
  end
end

function C:showIssuesWindow()
  editor.showWindow('mission_issues')
end



function C:calculateMissionIssues(missionList, windows, missionTypeWindow)
  self.issues = {list = {}}
  local idx = 1
  for _, mission in ipairs(missionList) do
    if not shipping_build and mission.devMission then
      -- skip mission
    else
      missionTypeWindow:setMission(mission)
      mission._issueList = {list = {}, importantCount = 0, highestSeverity = 'unknown'}

      for _, w in ipairs(windows) do
        if w.getMissionIssues then
          for _, issue in ipairs(w:getMissionIssues(mission) or {}) do

            issue.label = issue.label or "Unknown Issue"
            issue.severity = issue.severity or "unknown"
            issue.idx = idx
            idx = idx+1

            issue.missionId = mission.id
            issue.missionType = mission.missionType
            issue.level = mission.startTrigger.level or "None"
            issue.availability = (mission.careerSetup.showInFreeroam and "Freeroam " or "") .. (mission.careerSetup.showInCareer and "Career" or "")
            table.insert(self.issues.list, issue)
            table.insert(mission._issueList, issue)
            if issue.severity == 'warning' or issue.severity == 'error' then
              mission._issueList.importantCount = mission._issueList.importantCount + 1
            end
            if severityPriority[mission._issueList.highestSeverity] < severityPriority[issue.severity] then
              mission._issueList.highestSeverity = issue.severity
            end
          end
        end
        if mission._issueList.importantCount == 0 then
          mission._issueList.icon = 'check'
          mission._issueList.color = im.ImVec4(0, 1, 0, 1.0)
        else
          local c = math.min(mission._issueList.importantCount, 10)
          mission._issueList.color = im.ImVec4(0.8+c*0.02, 0.8-0.08*c, 0, 1.0)
          mission._issueList.icon = 'warning'
        end
      end
    end
  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
