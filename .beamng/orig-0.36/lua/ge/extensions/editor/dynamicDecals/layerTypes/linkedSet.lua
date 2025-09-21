-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerTypes_linkedSet"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local inspector = nil
local docs = nil
local widgets = nil

local function setProperty(layer, property)
  local propType = api.propertiesMap[property.id].type

  if propType == api.types.bool then
    layer[property.id] = property.value
  elseif propType == api.types.int or propType == api.types.float then
    layer[property.id] = property.value
  elseif propType == api.types.Point2F then
    layer[property.id] = Point2F.fromTable(property.value)
  elseif propType == api.types.Point3F then
    layer[property.id] = vec3(property.value[1], property.value[2], property.value[3])
  elseif propType == api.types.Point4F then
    layer[property.id] = Point4F.fromTable(property.value)
  elseif propType == api.types.string then
    layer[property.id] = property.value
  elseif propType == api.types.Texture then
    layer[property.id] = property.value
  elseif propType == api.types.File then
    layer[property.id] = property.value
  elseif propType == api.types.ColorI then
    layer[property.id] = ColorI.fromTable(property.value)
  else
    print("not yet implemented")
  end
end

local function setPropertiesInChildrenRec(layer, properties)
  if not layer.children then return end

  for _, child in ipairs(layer.children) do
    for _, property in pairs(properties) do

      if child[property.id] ~= nil then

        if property.id == "decalGradientColor" then
          local prop = {
            id = "decalGradientColorTopLeft",
            value = property.value[1]
          }
          setProperty(child, prop)
          prop = {
            id = "decalGradientColorTopRight",
            value = property.value[2]
          }
          setProperty(child, prop)
          prop = {
            id = "decalGradientColorBottomLeft",
            value = property.value[3]
          }
          setProperty(child, prop)
          prop = {
            id = "decalGradientColorBottomRight",
            value = property.value[4]
          }
          setProperty(child, prop)

        -- special case of 'useSurfaceNormal'
        elseif property.id == "useSurfaceNormal" then
          -- ALERT
          -- This is a hack. The decal matrix is not recalculated for this layer as long as it has the 'decalPos' and 'decalNorm' field.
          child.decalPos = nil
          child.decalNorm = nil
          setProperty(child, property)
        else
          setProperty(child, property)
        end
      end
    end

    setPropertiesInChildrenRec(child, properties)
  end
end

local function inspectLayerGui(layer, guiId)
  if im.BeginPopup(string.format("%s_%s_AddPropertyPopup", layer.uid, guiId)) then
    for cat, properties in pairs(api.properties) do
      im.TextColored(editor.color.beamng.Value, cat)
      local props = deepcopy(properties)
      table.sort(props, function(a, b) return a.name < b.name end)
      for _, property in ipairs(props) do
        if not layer.properties[property.id] then
          if im.Selectable1(string.format("%s##LinkedSet_AddProperty_%s_%s_%s", property.name, cat, guiId, layer.uid)) then
            if layer.properties[property.id] then
              editor.logWarn(string.format("%s: %s", logTag, "Can't add the same property twice"))
              return
            end

            local prop = {
              id = property.id,
              value = shallowcopy(property.default),
            }
            layer.properties[prop.id] = prop
            layer.propertiesDirty = true
            api.setLayer(layer, false)
          end
        end
      end
    end
    im.EndPopup()
  end

  im.TextColored(editor.color.beamng.Value, "Properties")
  im.SameLine()
  -- local btnSize = im.GetFontSize() + 2 * im.GetStyle().FramePadding.y
  -- if editor.uiIconImageButton(editor.icons.add, im.ImVec2(btnSize, btnSize), nil, nil, nil, string.format("PropertiesAddButton_%s_%s", guiId, layer.uid)) then
  --   im.OpenPopup(string.format("%s_%s_AddPropertyPopup", layer.uid, guiId))
  -- end
  -- im.tooltip("Add property")

  if im.Button(string.format("Add##LinkedSet_%s_%s", guiId, layer.uid), im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
    im.OpenPopup(string.format("%s_%s_AddPropertyPopup", layer.uid, guiId))
  end
  im.tooltip("Add property")

  local sortedKeys = tableKeys(layer.properties)
  table.sort(sortedKeys)

  for _, id in ipairs(sortedKeys) do
    local property = layer.properties[id]
    -- dump(property)
    if editor.uiIconImageButton(editor.icons.delete, tool.getIconSizeVec2(), nil, nil, nil, string.format("PropertiesRemoveButton_%s_%s_%s", guiId, layer.uid, property.id)) then
      layer.properties[property.id] = nil
    end
    im.SameLine()
    if widgets.draw(property.value, api.propertiesMap[property.id], string.format("%s_properties_%s", layer.uid, guiId), editor.getTempBool_BoolBool(false)) then
      property.value = api.propertiesMap[property.id].value
    end
    if editor.getTempBool_BoolBool() then
      property.value = api.propertiesMap[property.id].value
      layer.propertiesDirty = true
    end
  end

  im.Separator()
  if im.Button(string.format("Apply##%s_%s_LinkedSetProperties", layer.uid, guiId), im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
    setPropertiesInChildrenRec(layer, layer.properties)
    layer.propertiesDirty = false
    api.setLayer(layer, true)
  end
  if layer.propertiesDirty then
    im.Dummy(im.ImVec2(0, im.GetStyle().ItemSpacing.y * 2))
    if im.BeginChild1("PropertiesDirtyInfoChild", nil, true) then
      im.PushTextWrapPos(im.GetContentRegionAvailWidth())
      im.TextUnformatted("Properties have been altered and the changes have not been applied yet.\nHit the 'Apply' button in order to propagate the values to all child layers.")
      im.PopTextWrapPos()
    end
    im.EndChild()
  end
end

local function toolbarItemGui()
  if editor.uiIconImageButton(editor.icons.link, nil, nil, nil, nil, "Add linked set") then
    api.addLinkedSet()
  end
  im.tooltip("Add linked set layer.\nA linked set sets the properties of all child layers to a certain value.\nProperties can be added in the inspector.")
end

local function sectionGui(guiId)

end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Linked Set Layers introduce a powerful way to streamline the management of multiple layers.

These layers function similarly to groups.
When a Linked Set Layer is selected though, properties can be added in the Inspector and the values of these can be edited.

Once the 'Apply' button is hit the values of the added properties are propagated recursively to all child layers, irrespective of their types.
Consequently, numerous layers are updated in unison, enabling you to efficiently alter properties across a multitude of layers with just one action.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  inspector = extensions.editor_dynamicDecals_inspector
  docs = extensions.editor_dynamicDecals_docs
  widgets = extensions.editor_dynamicDecals_widgets

  -- tool.registerSection("Linked Set Properties", sectionGui, 110, false, {})
  inspector.registerLayerGui(api.layerTypes.linkedSet, inspectLayerGui)
  tool.registerToolbarToolItem("linkedSet", toolbarItemGui, 40)
  docs.register({section = {"Layer Types", "Linked Set Layers"}, guiFn = documentationGui})
end

M.onGui = sectionGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M