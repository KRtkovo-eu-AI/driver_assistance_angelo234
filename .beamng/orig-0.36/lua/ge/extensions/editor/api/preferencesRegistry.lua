-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

local DefaultPreferencesPath = "settings/editor/preferences.json"
local PreferencesFileFormatVersion = 2

function C:itemValueFromString(item, itemVal)
  if item.type == "float"
    or item.type == "int"
    or item.type == "double"
    or item.type == "char"
  then
    itemVal = tonumber(itemVal)
  elseif item.type == "bool" then
    itemVal = (itemVal == "true")
  elseif item.type == "table" then
    itemVal = jsonDecode(itemVal)
  elseif item.type == "Point3F" or item.type == "vec3" then
    itemVal = vec3:fromString(itemVal)
  elseif item.type == "Point2F" then
    itemVal = Point2F.fromString(itemVal)
  elseif item.type == "Point2I" then
    itemVal = Point2I.fromString(itemVal)
  elseif item.type == "Point4F" then
    itemVal = Point4F.fromString(itemVal)
  elseif item.type == "ColorI" then
    itemVal = ColorI.fromString(itemVal)
  elseif item.type == "ColorF" then
    itemVal = ColorF.fromString(itemVal)
  elseif item.type == "RectF" then
    itemVal = RectF.fromString(itemVal)
  end

  return itemVal
end

function C:itemValueToString(item, itemVal)
  if item.type == "float" or
    item.type == "int" or
    item.type == "double" or
    item.type == "char" or
    item.type == "bool" then
      itemVal = tostring(itemVal)
  elseif item.type == "table" then
    -- if the item value is a string, then we've already have an encoded json
    if type(itemVal) == "string" then return itemVal end
    itemVal = jsonEncode(itemVal)
  elseif item.type == "Point3F" or
    item.type == "vec3" or
    item.type == "Point2F" or
    item.type == "Point2I" or
    item.type == "Point4F" or
    item.type == "ColorI" or
    item.type == "ColorF" or
    item.type == "RectF"
  then
    itemVal = tostring(itemVal)
    -- remove comma
    itemVal = itemVal:gsub(",", "")
  end

  return itemVal
end

function C:clear()
  self.categories = {}
  self.nonPersistentItemPaths = {}
  self.preferences = nil
  self.changedItems = {}
  self.dirty = false
end

function C:init()
  self:clear()
end

function C:registerCategory(categName, categLabel)
  local cat = self:findCategory(categName)
  local foundCat = cat ~= nil
  categLabel = categLabel or string.sentenceCase(categName)

  if not cat then
    cat = {name = categName, label = categLabel, subcategories = {}}
    table.insert(self.categories, cat)
  end

  if foundCat and cat.label ~= categLabel then
    editor.logWarn("Preferences Registry: category label is different from other function calls, replaced: '" .. cat.label .. "' with: '" .. categLabel .. "'")
    cat.label = categLabel
  end

  return cat
end

function C:registerSubCategory(categName, subCategName, subCategLabel, prefs)
  local cat = self:findCategory(categName)
  subCategLabel = subCategLabel or string.sentenceCase(subCategName)

  if not cat then
    editor.logError("Preferences Registry: category '" .. categName .. "' not registered, please use registerCategory first (trying to add subcategory '" .. subCategName .. "')")
    return
  end

  local subCateg = self:findSubCategory(cat, subCategName)
  local foundSubCat = subCateg ~= nil

  if not subCateg then
    subCateg = {name = subCategName, label = subCategLabel, items = {}}
    table.insert(cat.subcategories, subCateg)
  end

  if foundSubCat and subCateg.label ~= subCategLabel then
    editor.logWarn("Preferences Registry: category label is different from other function calls, replaced: '" .. subCateg.label .. "' with: '" .. subCategLabel .. "'")
    subCateg.label = subCategLabel
  end

  if prefs then
    self:registerPreferences(categName, subCategName, prefs)
  end

  return subCateg
end

function C:addNonPersistentItemPath(subCategoryPath)
  self.nonPersistentItemPaths[subCategoryPath] = subCategoryPath
end

function C:removeNonPersistentItemPath(subCategoryPath)
  self.nonPersistentItemPaths[subCategoryPath] = nil
end

function C:registerPreferences(categName, subCategName, prefs)
  local cat = self:findCategory(categName)
  if not cat then
    editor.logError("Preferences Registry: no category found: " .. categName)
    return
  end
  local subCateg = self:findSubCategory(cat, subCategName)
  if not subCateg then
    editor.logError("Preferences Registry: no category/subcategory path found for: " .. categName .. "." .. subCategName)
    return
  end
  for _, pref in pairs(prefs) do
    -- we only have a single pair in the table, the table is used to keep the item order like it was written in the code
    local prefName, prefInfo = next(pref)
    --TODO: validate the fields in prefInfo, could find errors in type
    local item = self:findItemInSubCategory(subCateg, prefName)

    if not item then
      -- we are creating a table for enum combo in valueInspector, ready to use, if we have some enumLabels
      local enumTable = {}
      if prefInfo[10] then
        for i, val in ipairs(prefInfo[10]) do
          table.insert(enumTable, {name = val, value = i})
        end
      end
      -- if the prefInfo[4] is nil, we will automatically create a label from the name, with a "Pascal Case" format
      if not prefInfo[4] then
        prefInfo[4] = string.sentenceCase(prefName)
      end

      table.insert(subCateg.items,
        {
          name = prefName,
          path = categName .. "." .. subCategName .. "." .. prefName,
          type = prefInfo[1],
          defaultValue = prefInfo[2],
          description = prefInfo[3] or "",
          label = prefInfo[4],
          minValue = prefInfo[5],
          maxValue = prefInfo[6],
          hidden = prefInfo[7] or false,
          advanced = prefInfo[8] or false,
          customUiFunc = prefInfo[9],
          enumLabels = prefInfo[10],
          version = prefInfo[11] or 0,
          enum = enumTable
      })
    else
      editor.logError("Preferences Registry: duplicate item, already added: " .. categName .. "." .. subCategName .. "." .. prefName)
    end
  end
end

function C:findCategory(name)
  for _, cat in ipairs(self.categories) do
    if cat.name == name then return cat end
  end
end

function C:findSubCategory(category, name)
  for _, subcat in ipairs(category.subcategories) do
    if subcat.name == name then return subcat end
  end
end

function C:findItem(itemPath)
  local str = split(itemPath, ".")
  if #str < 3 then return nil end
  local prefCat = str[1]
  local prefSubCat = str[2]
  local prefName = str[3]
  local cat = self:findCategory(prefCat)
  if not cat then return nil end
  local subcat = self:findSubCategory(cat, prefSubCat)
  if not subcat then return nil end
  if type(subcat) ~= "table" then return nil end
  for _, item in ipairs(subcat.items) do
    if item.name == prefName then return item end
  end
end

function C:findItemInSubCategory(subCateg, itemName)
  if not subCateg then return nil end
  for _, item in ipairs(subCateg.items) do
    if item.name == itemName then return item end
  end
end

function C:migratePreferences(deleteDeprecatedItems)
  if self.preferences == nil then return end
  -- check the current preferences.json items if they still exist in the registry
  for catName, cat in pairs(self.preferences) do
    if type(cat) ~= "table" then
      editor.logWarn("Preferences category " .. catName .. " is invalid, not a table, ignoring, using defaults")
      cat = {} -- just make an empty one
    else
      for subcatName, subcat in pairs(cat) do
        if type(subcat) ~= "table" then
          editor.logWarn("Preferences subcategory " .. catName .. "." .. subcatName .. " is invalid, not a table, ignoring, using defaults")
          self.preferences[catName][subcatName] = {} -- just make an empty one
        else
          for itemName, item in pairs(subcat) do
            local itemPath = catName .. "." .. subcatName .. "." .. itemName
            local itemInRegistry = self:findItem(catName .. "." .. subcatName .. "." .. itemName)
            if not itemInRegistry then
              local ret = extensions.hook("onEditorDeprecatedPreferencesItem", itemPath, self.preferences[catName][subcatName][itemName])
              -- delete deprecated pref value
              if deleteDeprecatedItems or ret == true then
                editor.logWarn("Preferences Registry: deprecated preferences item found: " .. itemPath .. ", migrated and deleted old item...")
                self.preferences[catName][subcatName][itemName] = nil
              end
            end
          end
        end
      end
    end
  end
end

function C:set(itemPath, value)
  local str = split(itemPath, ".")
  if #str < 3 then return nil end
  local prefCat = str[1]
  local prefSubCat = str[2]
  local prefName = str[3]

  if not self.preferences[prefCat] then self.preferences[prefCat] = {} end
  if not self.preferences[prefCat][prefSubCat] then self.preferences[prefCat][prefSubCat] = {} end

  local item = self:findItem(itemPath)
  if not item then return editor.logError("No registered preference: " .. itemPath) end
  if item and type(value) == "number" then
    value = clamp(value, item.minValue or -math.huge, item.maxValue or math.huge)
  end

  self.preferences[prefCat][prefSubCat][prefName] = value
  self.dirty = true

  if not self.dontCallHookForSetValue then
    table.insert(self.changedItems, {itemPath = itemPath, value = value})
  end
end

-- call the preference changed hooks and return true if we had any changed items
function C:broadcastPreferenceValueChanged()
  if 0 == #self.changedItems then return false end
  self.dontCallHookForSetValue = true
  for _, item in ipairs(self.changedItems) do
    extensions.hook("onEditorPreferenceValueChanged", item.itemPath, item.value)
  end
  self.dontCallHookForSetValue = false
  self.changedItems = {}
  return true
end

-- use this to get a preference item's value by path
-- you can also use also more directly: editor.getPreference("someCategory.someSubCategory.someItemName")
function C:get(itemPath)
  local str = split(itemPath, ".")
  if #str < 3 then return nil end
  local prefCat = str[1]
  local prefSubCat = str[2]
  local prefName = str[3]

  if self.preferences[prefCat] == nil then return nil end
  if self.preferences[prefCat][prefSubCat] == nil then return nil end
  if self.preferences[prefCat][prefSubCat][prefName] == nil then return nil end
  return self.preferences[prefCat][prefSubCat][prefName]
end

function C:getAsString(itemPath)
  local item = self:findItem(itemPath)
  return self:itemValueToString(item, self:get(itemPath))
end

function C:loadPreferences(filename)
  local fname = filename or DefaultPreferencesPath
  self.preferences = deepcopy(jsonReadFile(fname)) or {}
  if self.preferences.version ~= PreferencesFileFormatVersion then
    -- different version, ignore, use defaults
    editor.logWarn("Deprecated preferences file format, using default values.")
    self.preferences = {}
  end
  self.preferences.version = nil -- just delete version, not needed
  self.dontCallHookForSetValue = true
  -- fill all items with default values so its easy to lookup without search
  for _, cat in ipairs(self.categories) do
    for _, subCat in ipairs(cat.subcategories) do
      for _, item in ipairs(subCat.items) do
        local path = cat.name .. "." .. subCat.name .. "." .. item.name
        local val = nil
        local isDefaultValue = false

        -- get the preference value from the file (which is right now in self.preferences)
        -- if the category doesnt exist in the file, use the default value
        if nil == self.preferences[cat.name] then
          val = deepcopy(item.defaultValue)
          isDefaultValue = true
        elseif type(self.preferences[cat.name]) ~= "table" then
          -- skip, category should be a table
        elseif nil == self.preferences[cat.name][subCat.name] then
          -- default, subcategory should be non nil
          val = deepcopy(item.defaultValue)
          isDefaultValue = true
        elseif type(self.preferences[cat.name][subCat.name]) ~= "table" then
          -- skip, subcategory should be a table
        elseif nil == self.preferences[cat.name][subCat.name][item.name] then
          -- default value, was not found in the json file
          val = deepcopy(item.defaultValue)
          isDefaultValue = true
        else
          -- the value in json file is used
          val = self.preferences[cat.name][subCat.name][item.name]
        end

        local isOldValue = false

        if val ~= nil then
          -- old pref values are strings or in some cases numbers (even older prefs)
          -- so check if its not a table (the new version of prefs are tables)
          isOldValue = type(val) ~= "table" and not isDefaultValue
          -- check if the value is an old preference string value we need to convert to a table with type and version
          if isOldValue then
            val = {
              oldValue = val, -- keep the file value for onEditorPreferenceVersionChanged
              value = self:itemValueFromString(item, tostring(val)), -- try to convert (if type stayed the same)
              type = item.type,
              version = 0
            }
          elseif isDefaultValue then
            val = {
              value = val,
              type = item.type,
              version = item.version
            }
          else
            -- check version, if changed, allow user to do the migration
            if item.version ~= val.version then
              extensions.hook("onEditorPreferenceVersionChanged", path, val, item)
            end
            -- convert the file string value to actual type
            val = {
              value = self:itemValueFromString(item, val.value),
              type = item.type,
              version = item.version
            }
          end
        end

        if nil ~= val then
          if nil ~= val.value then
            -- set the value again, if set by user or default
            self:set(path, val.value)
          else
            editor.logWarn("Preferences: Could not set default value for: `" .. path .. "`")
            dump(val)
          end
        else
          editor.logWarn("Preferences: Could not set actual default value (it's nil) for: `" .. path .. "`")
          dump(val)
        end
      end
    end
  end
  self.dontCallHookForSetValue = false
  self.dirty = false
end

function C:callHookForSetValue()
  for _, cat in ipairs(self.categories) do
    for _, subCat in ipairs(cat.subcategories) do
      for _, item in ipairs(subCat.items) do
        local path = cat.name .. "." .. subCat.name .. "." .. item.name
        local itemVal = self:get(path)
        extensions.hook("onEditorPreferenceValueChanged", path, itemVal)
      end
    end
  end
end

function C:loadCategory(categoryName, filename)
  local json = jsonReadFile(filename) or {}

  if json.version ~= PreferencesFileFormatVersion then
    -- different version, ignore, use defaults
    editor.logWarn("Deprecated category preferences file format, not loading.")
    return
  end
  json.version = nil -- just delete version, no needed

  local cat = self:findCategory(categoryName)
  if cat then
    if json[categoryName] then
      for subCatName, subCat in pairs(json[categoryName]) do
        for itemName, itemValue in pairs(subCat) do
          local path = categoryName .. "." .. subCatName .. "." .. itemName
          local item = self:findItem(path)
          self:set(path, self:itemValueFromString(item, itemValue.value))
        end
      end
      self.dirty = true
    end
  end
end

-- Save a single category from the preferences, it will save all the preferences in the category, regardless if default values or not
-- @param categoryName the category (along with all the subcategories) to be saved
-- @param filename the json filename to save to
function C:saveCategory(categoryName, filename)
  local prefsToSave = {}
  prefsToSave.version = PreferencesFileFormatVersion
  local val = nil
  for subcatName, subcat in pairs(self.preferences[categoryName]) do
    for itemName, item in pairs(subcat) do
      local itemPath = categoryName .. "." .. subcatName .. "." .. itemName
      local itemInRegistry = self:findItem(itemPath)
      local isPrefValueNonPersistent = self.nonPersistentItemPaths[itemPath]
      if itemInRegistry and not isPrefValueNonPersistent then
        val = self:itemValueToString(itemInRegistry, self.preferences[categoryName][subcatName][itemName])
        if not prefsToSave[categoryName] then prefsToSave[categoryName] = {} end
        if not prefsToSave[categoryName][subcatName] then prefsToSave[categoryName][subcatName] = {} end
        prefsToSave[categoryName][subcatName][itemName] = {
          value = val,
          type = itemInRegistry.type,
          version = itemInRegistry.version }
      end
    end
  end
  jsonWriteFile(filename, prefsToSave or {}, true)
end

function C:savePreferences(filename)
  local fname = filename or DefaultPreferencesPath
  -- we keep all preference value that were set in the past, because some extensions might not be loaded anymore but they might be loaded in the future
  local prefsToSave = deepcopy(self.preferences)
  if not prefsToSave then return false end
  prefsToSave.version = PreferencesFileFormatVersion
  -- prune prefs that have default values, no need to be in the file
  for catName, cat in pairs(self.preferences) do
    for subcatName, subcat in pairs(cat) do
      for itemName, _ in pairs(subcat) do
        local itemPath = catName .. "." .. subcatName .. "." .. itemName
        local item = self:findItem(itemPath)
        local prefValueAlreadyExistsInFile = (prefsToSave[catName] ~= nil and prefsToSave[catName][subcatName] ~= nil and prefsToSave[catName][subcatName][itemName] ~= nil)
        local isPrefValueNonPersistent = self.nonPersistentItemPaths[itemPath]
        if item and not isPrefValueNonPersistent then
          -- we convert to strings before compare, since comparing pointers (for some types) will yield wrong results
          local itemValue = self:itemValueToString(item, self.preferences[catName][subcatName][itemName])
          local itemDefaultValue = self:itemValueToString(item, item.defaultValue)
          local isDefaultValue = itemValue == itemDefaultValue

          if not isDefaultValue then
            if not prefsToSave[catName] then prefsToSave[catName] = {} end
            if not prefsToSave[catName][subcatName] then prefsToSave[catName][subcatName] = {} end
            prefsToSave[catName][subcatName][itemName] = {
              value = self:itemValueToString(item, self.preferences[catName][subcatName][itemName]),
              type = item.type,
              version = item.version }
          else
            -- its a default value, no need to save it in the file, delete it from the table
            prefsToSave[catName][subcatName][itemName] = nil
          end
        end
      end
      -- if there are no prefs in a subcategory, delete it from the table
      if prefsToSave[catName] and prefsToSave[catName][subcatName] and tableIsEmpty(prefsToSave[catName][subcatName]) then
        prefsToSave[catName][subcatName] = nil
      end
    end
    -- if there are no subcategories in a category, delete it from the table
    if prefsToSave[catName] and tableIsEmpty(prefsToSave[catName]) then
      prefsToSave[catName] = nil
    end
  end

  jsonWriteFile(fname, prefsToSave or {}, true)
  self.dirty = false
end

function C:resetToDefaults(categoryName)
  if categoryName == nil then
    -- Recursive all-categories mode
    for _, cat in ipairs(self.categories) do
      self:resetToDefaults(cat.name)
    end
  else
    --  Single category mode
    local cat = self:findCategory(categoryName)
    if not cat then return false end
    for _, subCat in pairs(cat.subcategories) do
      for _, item in pairs(subCat.items) do
        self:set(item.path, deepcopy(item.defaultValue))
      end
    end
  end
  return true
end

function C:resetItemToDefault(itemPath)
  local item = self:findItem(itemPath)
  self:set(item.path, deepcopy(item.defaultValue))
  return true
end

return function()
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init()
  return o
end