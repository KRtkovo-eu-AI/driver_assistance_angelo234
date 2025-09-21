-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
local im = ui_imgui

local _uid = 0 -- internal use only
local function getNextUniqueIdentifier()
  _uid = _uid + 1
  return _uid
end

local paintNameKeys = {"paintName", "paintName2", "paintName3"}
local defaultPaint = {1, 1, 1, 1}
local imDummy = im.ImVec2(0, 5)
local defaultColumnWidth = 80

function C:init(name, data)
  self.id = getNextUniqueIdentifier()
  self.name = name or "Vehicle"

  self:resetSelections()
  self:resetOptions()

  if data then
    self:setModel(data.model)
    self:setConfig(data.config)
    if data.customConfigPath or data.customConfigActive then
      self:resetConfig()
      self.customConfigPath = data.customConfigActive and data.configPath or data.customConfigPath
      self.configPath = self.customConfigPath
      self.customConfigActive = true
    end

    for i, key in ipairs(paintNameKeys) do
      self[key] = data[key]
    end
  end
end

function C:setModel(model) -- directly sets the model data, by key
  if not model then return end

  self.model = model
  local modelData = core_vehicles.getModel(self.model)
  if modelData and modelData.model then
    self.vehType = modelData.model.Type
    self.modelName = modelData.model.Name
  else
    self.model = nil
  end
end

function C:setConfig(config) -- directly sets the config data, by key (self.model needs to exist)
  if not config or not self.model then return end

  self.config = config
  local modelData = core_vehicles.getModel(self.model)
  if modelData and modelData.model then
    local configData = modelData.configs[self.config]
    if configData then
      self.configName = configData.Name
      self.configPath = "vehicles/"..self.model.."/"..self.config..".pc"
    else
      self.config = nil
    end
  else
    self.config = nil
  end
end

function C:setOptions(data) -- sets some specific options for this utility
  if not data then return end

  for _, key in ipairs({"enableConfigs", "enablePaints", "enableCustomConfig", "paintLayers", "columnWidth"}) do
    if data[key] then
      self[key] = data[key]
    end
  end
end

function C:resetModel() -- resets all model data
  self.models = nil
  self.model = nil
  self.modelName = nil
end

function C:resetConfig() -- resets all config data
  self.configs = nil
  self.config = nil
  self.configName = nil
  self.configPath = nil
  self.customConfigPath = nil
  self.customConfigActive = false
  self.customConfigPtr = nil
end

function C:resetPaint() -- resets all paint data
  self.paints = nil
  self.paintKeys = nil
  self.paintName = nil
  self.paintName2 = nil
  self.paintName3 = nil
end

function C:resetSelections() -- resets the vehicle selection
  self.vehType = nil

  self:resetModel()
  self:resetConfig()
  self:resetPaint()
end

function C:resetOptions() -- resets the util options
  self.allowedTypes = {"Car", "Truck", "Automation", "Trailer", "Prop", "Utility", "Traffic", "Unknown", "Any"}
  self.allowedSubtypes = {"PropTraffic", "PropParked"} -- subtypes are for configs
  self.modelBlacklist = {}
  self.configBlacklist = {}

  self.enableConfigs = true
  self.enablePaints = false
  self.enableCustomConfig = false
  self.paintLayers = 3
end

function C:setModelListByField(key, value, disallow) -- whitelists (or blacklists) vehicle models by field name and value
  -- TODO
end

function C:setConfigListByField(model, key, value, disallow) -- whitelists (or blacklists) vehicle configs by field name and value (per model)
  -- TODO
end

function C:getPaintPtr(paintTbl) -- returns the imgui paint table for the given paint
  local newPaintTbl

  if type(paintTbl) == "table" then
    if paintTbl.baseColor then
      if paintTbl.baseColor[4] then
        newPaintTbl = deepcopy(paintTbl.baseColor)
      elseif paintTbl.baseColor.x and paintTbl.baseColor.y and paintTbl.baseColor.z and paintTbl.baseColor.w then
        newPaintTbl = {paintTbl.baseColor.x, paintTbl.baseColor.y, paintTbl.baseColor.z, paintTbl.baseColor.w}
      end
    else
      newPaintTbl = deepcopy(paintTbl)
    end
  else
    newPaintTbl = deepcopy(defaultPaint)
  end

  return editor.getTempFloatArray4_TableTable(newPaintTbl)
end

function C:fetchData() -- retrieves model, config, and paint data, whenever required
  if not self.vehType then
    self.vehType = self.allowedTypes[1] or "Any"
  end

  if not self.models then -- refresh models list, using the current vehicle type
    self.models = core_vehicles.getModelList(true).models
    for i = #self.models, 1, -1 do
      if (self.vehType ~= "Any" and self.models[i].Type ~= self.vehType) or self.modelBlacklist[self.models[i].key] then
        table.remove(self.models, i)
      end
    end

    table.sort(self.models, function(m1, m2)
      return (m1.Name or m1.key) < (m2.Name or m2.key)
    end)
  end

  if self.enableConfigs and self.model and not self.configs then -- refresh configs list, using the current model
    local modelData = core_vehicles.getModel(self.model)
    if modelData and modelData.model then
      self.modelName = dumps(modelData.model.Name)

      self.configs = {}
      for k, v in pairs(modelData.configs) do
        table.insert(self.configs, v)
      end

      for i = #self.configs, 1, -1 do
        local bList = self.configBlacklist[self.model]
        local cType = self.configs[i].Type
        if (bList and bList[self.configs[i].key]) or (cType and not arrayFindValueIndex(self.allowedSubtypes or {}, cType)) then
          table.remove(self.configs, i)
        end
      end

      table.sort(self.configs, function(m1, m2)
        return (m1.Name or m1.key) < (m2.Name or m2.key)
      end)
    end
  end

  if self.enablePaints and self.model and not self.paints then -- refreshes paints list, using the current model
    local modelData = core_vehicles.getModel(self.model)
    if modelData and modelData.model then
      self.paints = modelData.model.paints
      self.paintKeys = self.paints and tableKeysSorted(self.paints) or {}
    end
  end
end

function C:widget() -- displays the interface
  local updateReason = nil

  self:fetchData()

  local elemHeight = im.GetFrameHeightWithSpacing()
  local elemPadding = 10
  local elemCount = 2
  if self.enableConfigs then
    elemCount = elemCount + 1
  end
  if self.enablePaints then
    elemCount = elemCount + self.paintLayers
    elemPadding = elemPadding + 10
  end
  if self.enableCustomConfig then
    elemCount = elemCount + 2
    elemPadding = elemPadding + 10
  end

  local columnWidth = self.columnWidth or defaultColumnWidth
  columnWidth = columnWidth * im.uiscale[0]

  im.BeginChild1("##vehicleSelector"..dumps(self.id), im.ImVec2(im.GetContentRegionAvailWidth(), elemCount * elemHeight + elemPadding))

  im.Columns(2)
  im.SetColumnWidth(0, columnWidth)

  im.Text("Type")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())

  if not self.allowedTypes[2] then
    if not self.vehType then
      self.vehType = self.allowedTypes[1] or "Any"
      updateReason = "type"
    end
    im.BeginDisabled()
  end

  if im.BeginCombo("##vehicleSelectorTypes"..dumps(self.id), self.vehType) then
    for _, t in ipairs(self.allowedTypes) do
      if im.Selectable1(t.."##vehicleSelectorTypeNames"..dumps(self.id), t == self.vehType) then
        self.vehType = t
        updateReason = "type"
      end
    end
    im.EndCombo()
  end

  if not self.allowedTypes[2] then im.EndDisabled() end
  im.PopItemWidth()
  im.NextColumn()

  im.Text("Model")
  im.NextColumn()

  local label = "(None)"
  if self.model then
    label = dumps(self.modelName).." ["..self.model.."]"
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())

  local imDisabled = false
  if self.models and not self.models[2] then -- only one model exists
    if not self.model then
      local modelData = self.models[1]
      if modelData and modelData.key then
        self.model = modelData.key
        self.modelName = modelData.Name
        updateReason = "model"
      end
    end
    im.BeginDisabled()
    imDisabled = true
  end

  if im.BeginCombo("##vehicleSelectorModels"..dumps(self.id), label) then
    for _, m in ipairs(self.models) do
      local label = m.Name and (m.Name.." ["..m.key.."]")
      if im.Selectable1(label.."##vehicleSelectorModelNames"..dumps(self.id) or m.key, m.key == self.model) then
        self.model = m.key
        self.modelName = m.Name
        updateReason = "model"
      end
    end
    im.EndCombo()
  end

  if imDisabled then im.EndDisabled() end
  im.PopItemWidth()
  im.NextColumn()

  if self.enableConfigs then
    im.Text("Config")
    im.NextColumn()

    local imDisabled = false
    if not self.configs or self.customConfigPath then
      im.BeginDisabled()
      imDisabled = true
    elseif self.configs and not self.configs[2] then -- only one config exists
      if not self.config then
        local configData = self.configs[1]
        if configData and configData.key then
          self.config = configData.key
          self.configName = configData.Name
          self.configPath = "vehicles/"..self.model.."/"..self.config..".pc"
          updateReason = "config"
        end
      end
      im.BeginDisabled()
      imDisabled = true
    end

    local label = "(None)"
    if self.configs then
      label = self.config and dumps(self.configName).." ["..self.config.."]" or "(Default)"
    end

    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.BeginCombo("##vehicleSelectorConfigs"..dumps(self.id), label) then
      if self.configs then
        local modelData = core_vehicles.getModel(self.model)
        local modelConfigs = modelData and modelData.configs or {}

        if im.Selectable1("(Default)##vehicleSelectorConfigNames"..dumps(self.id), self.config == nil) then
          self.config = nil
          self.configName = nil
          self.configPath = nil
          updateReason = "config"
        end

        for _, c in ipairs(self.configs) do
          local label = dumps(c.Name).." ["..dumps(c.key).."]"
          if im.Selectable1(label.."##vehicleSelectorConfigNames"..dumps(self.id), c.key == self.config) then
            self.config = c.key
            self.configName = c.Name
            self.configPath = "vehicles/"..self.model.."/"..self.config..".pc"
            updateReason = "config"
          end
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    if imDisabled then im.EndDisabled() end
    im.NextColumn()
  end

  if self.enablePaints then
    im.Columns(1)
    im.Dummy(imDummy)
    im.Columns(2)
    im.SetColumnWidth(0, columnWidth)

    local isHovered = false
    self._hoveredIdx = self._hoveredIdx or 0

    for i, key in ipairs(paintNameKeys) do
      if self.paintLayers and i <= self.paintLayers then
        if self.paintLayers == 1 then
          im.Text("Paint")
        else
          im.Text("Paint "..i)
        end

        if self.paints then
          im.SameLine()
          local currPaint = self.paints[self[key]] and self.paints[self[key]].baseColor
          if i == self._hoveredIdx then currPaint = self._hoveredPaint end
          im.ColorEdit4("##vehicleSelectorPaintPreview"..dumps(self.id).."_"..i, self:getPaintPtr(currPaint), im.flags(im.ColorEditFlags_NoPicker, im.ColorEditFlags_NoInputs))
        end
        im.NextColumn()

        if not self.paints then im.BeginDisabled() end

        local label = "(None)"
        if self.paints then
          label = self[key] or "(Default)"
        end

        im.PushItemWidth(im.GetContentRegionAvailWidth())
        if im.BeginCombo("##vehicleSelectorPaints"..dumps(self.id).."_"..i, label) then
          if im.Selectable1("(Default)##vehicleSelectorPaintNames"..dumps(self.id).."_"..i, self[key] == nil) then
            self[key] = nil
            updateReason = key
          end

          if im.IsItemHovered() then
            isHovered = true
            self._hoveredIdx = i
            self._hoveredPaint = defaultPaint
          end

          for _, paint in ipairs(self.paintKeys) do
            if im.Selectable1(paint.."##vehicleSelectorPaintNames"..dumps(self.id).."_"..i, self[key] == paint) then
              self[key] = paint
              updateReason = key
            end

            if im.IsItemHovered() then
              isHovered = true
              self._hoveredIdx = i
              self._hoveredPaint = self.paints[paint] and self.paints[paint].baseColor
            end
          end

          im.EndCombo()
        end
        im.PopItemWidth()

        if not self.paints then im.EndDisabled() end
        im.NextColumn()
      end
    end

    if not isHovered then
      self._hoveredIdx = 0
      self._hoveredPaint = nil
    end
  end

  if self.enableCustomConfig then
    im.Columns(1)
    im.Dummy(imDummy)
    im.Columns(2)
    im.SetColumnWidth(0, columnWidth)

    im.TextWrapped("Custom Config (Optional)")
    im.NextColumn()

    if not self.customConfigPtr then
      self.customConfigPtr = im.ArrayChar(1024, self.customConfigPath or "")
    end

    im.PushItemWidth(im.GetContentRegionAvailWidth() - 40)
    if editor.uiInputFile("##vehicleSelectorCustom"..dumps(self.id), self.customConfigPtr, nil, nil, {{"Part config files", ".pc"}}, im.InputTextFlags_EnterReturnsTrue) then
      local customConfigPath = ffi.string(self.customConfigPtr)

      if customConfigPath == "" then
        self.customConfigPath = nil
        self.customConfigActive = false
      else
        self:resetConfig()
        self.customConfigPath = customConfigPath
        self.configPath = self.customConfigPath
        self.customConfigActive = true
      end
      updateReason = "customConfig"
    end
    im.PopItemWidth()
    im.NextColumn()
  end

  im.Columns(1)
  im.EndChild()

  -- here, selections get updated whenever required
  if updateReason == "type" then
    self:resetModel()
  end
  if updateReason == "model" or not self.model then
    self:resetConfig()
    self:resetPaint()
  end

  return updateReason
end

-- helper callbacks for the edit mode
function C:onActivate()
end

function C:onDeactivate()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end