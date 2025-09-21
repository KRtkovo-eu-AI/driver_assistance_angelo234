-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Generic Mission Text'
C.color = ui_flowgraph_editor.nodeColors.ui
C.icon = ui_flowgraph_editor.nodeIcons.ui
C.behaviour = { duration = true }
C.description = "Displays generic mission info."
C.category = 'repeat_instant'
C.author = 'BeamNG'

C.pinSchema = {
  {dir = 'in', type = 'string', name = 'title', description = "Title of the info", default="missions.missions.general.time", hardcoded = true},
  {dir = 'in', type = 'any', name = 'txt', description = "Text of the info"},
  {dir = 'in', type = 'string', name = 'icon', description = "Icon of the info", default = "info"},
  {dir = 'in', type = 'number', name = 'order', description = "Order of the info. Higher order will be shown rightmost."},
  {dir = 'in', type = 'string', name = 'category',  default = 'flowgraph',  description = "Category for the info. Only one message per category can be displayed. If not set, uses title.", hidden = true},
  {dir = 'in', type = 'string', name = 'style', description = "Display mode for the info.", hidden=true, hardcoded = true, default = "text"},
  {dir = 'in', type = 'bool', name = 'clear', description = "Removes the info.", hidden=true},

}
C.tags = {'string','util'}

function C:postInit()
  self.pinInLocal.style.hardTemplates = {
    {label = "Text", value = "text"},
    {label = "Time", value = "time"},
    {label = "Time with milliseconds", value = "timemillis"},
  }
  self.pinInLocal.title.hardTemplates = {}
  for _, k in ipairs({"time","recoveries"}) do
    local key = "missions.missions.general." .. k
    table.insert(self.pinInLocal.title.hardTemplates, {label=translateLanguage(key, k, true), value = key})
  end
end


function C:work()
  if (self.pinIn.title.value == nil and self.pinIn.txt.value == nil) or self.pinIn.clear.value then
    if self.pinIn.category.value == nil then
      ui_apps_genericMissionData.clearData()
    else
      ui_apps_genericMissionData.setData({category = self.pinIn.category.value or self.pinIn.title.value, clear = true})
    end
  else
    local data = {
      title = self.pinIn.title.value,
      txt = self.pinIn.txt.value,
      category = self.pinIn.category.value or self.pinIn.title.value,
      style = self.pinIn.style.value,
      order = self.pinIn.order.value,
      icon = self.pinIn.icon.value,
    }

    if self.pinIn.style.value == "time" then
      data.txt = string.format("%02d:%02d", math.floor(self.pinIn.txt.value / 60), math.floor(self.pinIn.txt.value % 60))
      data.minutes = string.format("%02d", math.floor(self.pinIn.txt.value / 60))
      data.seconds = string.format("%02d", math.floor(self.pinIn.txt.value % 60))
      data.style = "text"
    elseif self.pinIn.style.value == "timemillis" then
      data.txt = string.format("%02d:%02d.%03d", math.floor(self.pinIn.txt.value / 60), math.floor(self.pinIn.txt.value % 60), math.floor((self.pinIn.txt.value % 1) * 1000))
      data.minutes = string.format("%02d", math.floor(self.pinIn.txt.value / 60))
      data.seconds = string.format("%02d", math.floor(self.pinIn.txt.value % 60))
      data.milliseconds = string.format("%03d", math.floor((self.pinIn.txt.value % 1) * 1000))
      data.style = "text"
    elseif self.pinIn.style.value == "text" then
      data.txt = self.pinIn.txt.value
    end
    ui_apps_genericMissionData.setData(data)
  end
  self.mgr.modules.ui.genericMissionDataChanged = true
end


return _flowgraph_createNode(C)
