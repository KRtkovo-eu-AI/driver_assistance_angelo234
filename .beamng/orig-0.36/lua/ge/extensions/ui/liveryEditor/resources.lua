-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.dependencies = {"editor_api_dynamicDecals_textures"}
local texturesApi = extensions.editor_api_dynamicDecals_textures

-- Category properties
-- id
-- name
-- preview

-- Texture properties
-- filename
-- preview
-- type sdf or normal
-- category

local notifyListeners = function(hookName, hookData)
  local fullHookName = "liveryEditor_resources"
  if hookName then
    fullHookName = fullHookName .. "_" .. hookName
  end

  guihooks.trigger(fullHookName, hookData)
end

local function parseTextures(taggedTextures)
  M.textures = {}
  for tag, taggedTextures in pairs(taggedTextures) do
    local categorizedTextures = {
      value = tag,
      label = tag,
      items = {}
    }

    for key, file in pairs(taggedTextures) do
      local _, filename, _ = path.split(file)
      table.insert(categorizedTextures.items, {
        name = filename,
        label = filename,
        value = filename,
        preview = file
      })
    end

    table.insert(M.textures, categorizedTextures)
  end
end

M.textures = {}

M.setup = function()
  texturesApi.setup()
  local taggedTextures = texturesApi.getTagsWithRefs()
  parseTextures(taggedTextures)

  -- set textures without tags
  local items = {}

  for k, texture in ipairs(texturesApi.getTextureFiles()) do
    local sidecarFile = texturesApi.readSidecarFile(texture)
    if not sidecarFile or not sidecarFile.tags or #sidecarFile.tags == 0 then
      local _, filename, _ = path.split(texture)
      table.insert(items, {
        name = filename,
        label = filename,
        value = filename,
        preview = texture
      })
    end
  end

  if #items > 0 then
    table.insert(M.textures, {
      value = "others",
      label = "Others",
      items = items
    })
  end
end

M.requestData = function()
  notifyListeners("data", M.textures)
end

M.getTextureCategories = function()
  table.sort(M.textures, function(a, b)
    return a.label:lower() < b.label:lower()
  end)

  return M.textures
end

M.getTexturesByCategory = function(category)
  for key, textureCategory in ipairs(M.textures) do
    if textureCategory.value == category then
      return textureCategory
    end
  end
end

M.getDecalTextures = function()
  return M.textures
end

M.getCategories = function()
  return M.categories
end

M.dynamicDecals_onTextureFileAdded = function(textureFilePath)
end

M.dynamicDecals_onTextureFileDeleted = function(textureFilePath)
end

return M
