-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local _state = {
  vehicle = {
    nodeDebugTextTypeToID = {},
    nodeDebugTextMode = 1,
    nodeDebugTextModes = {
      {name = "off"},
    },
  },
  partsSelected = nil,
}

local bdebugImpl = nil

local function initBDebugImpl(bdebugImplSavedState)
  if not bdebugImpl then
    bdebugImpl = require('bdebugImpl')
    bdebugImpl.init(bdebugImplSavedState, _state)

    -- Redirect function calls to bdebugImpl.lua
    -- These functions can be nopped, hence the function wrapping
    M.nodeCollision = function(p) bdebugImpl.nodeCollision(p) end
    M.beamBroke = function(id, energy) bdebugImpl.beamBroke(id, energy) end
    M.debugDraw = function(focusPos) bdebugImpl.debugDraw(focusPos) end

    -- Rest aren't nopped
    M.isEnabled = bdebugImpl.isEnabled
    M.onPlayersChanged = bdebugImpl.onPlayersChanged
    M.reset = bdebugImpl.reset
    M.recieveViewportSize = bdebugImpl.recieveViewportSize

    M.setPartsSelected = bdebugImpl.setPartsSelected

    M.setNodeDebugText = bdebugImpl.setNodeDebugText
    M.clearNodeDebugText = bdebugImpl.clearNodeDebugText
    M.clearTypeNodeDebugText = bdebugImpl.clearTypeNodeDebugText
    M.clearAllNodeDebugText = bdebugImpl.clearAllNodeDebugText
  end
end


-- Following interfaces will initalize bdebugImpl

M.initState = {}
setmetatable(M.initState, {
  __index = function(_, key)
    initBDebugImpl()
    return bdebugImpl.initState[key]  -- Redirect read access to the current bdebugImpl.initState
  end,
  __newindex = function(_, key, value)
    initBDebugImpl()
    bdebugImpl.initState[key] = value  -- Redirect write access to the current bdebugImpl.initState
  end
})

M.state = {}
setmetatable(M.state, {
  __index = function(_, key)
    initBDebugImpl()
    return bdebugImpl.state[key]  -- Redirect read access to the current bdebugImpl.state
  end,
  __newindex = function(_, key, value)
    initBDebugImpl()
    bdebugImpl.state[key] = value  -- Redirect write access to the current bdebugImpl.state
  end
})

M.stateNoReset = {}
setmetatable(M.stateNoReset, {
  __index = function(_, key)
    initBDebugImpl()
    return bdebugImpl.stateNoReset[key]  -- Redirect read access to the current bdebugImpl.stateNoReset
  end,
  __newindex = function(_, key, value)
    initBDebugImpl()
    bdebugImpl.stateNoReset[key] = value  -- Redirect write access to the current bdebugImpl.stateNoReset
  end
})

local function sendState()
  initBDebugImpl()
  bdebugImpl.requestState()
end

local function setState(state, stateNoReset, notSendBack)
  initBDebugImpl()
  bdebugImpl.setState(state, stateNoReset, notSendBack)
end

-- Request/send drawn nodes to GE Lua function
local function requestDrawnNodesGE(geFuncName)
  initBDebugImpl()
  bdebugImpl.requestDrawnNodesGE(geFuncName)
end

-- Request/send drawn beams to GE Lua function
local function requestDrawnBeamsGE(geFuncName)
  initBDebugImpl()
  bdebugImpl.requestDrawnBeamsGE(geFuncName)
end

local function isEnabled()
  return false
end

local function setEnabled(enabled)
  initBDebugImpl()
  bdebugImpl.setEnabled(enabled)
end

-- User input events

-- function used by the input subsystem - AND NOTHING ELSE
-- DO NOT use these from the UI
local function toggleEnabled()
  initBDebugImpl()
  bdebugImpl.toggleEnabled()
end

local function nodetextModeChange(change)
  initBDebugImpl()
  bdebugImpl.nodetextModeChange(change)
end

local function nodevisModeChange(change)
  initBDebugImpl()
  bdebugImpl.nodevisModeChange(change)
end

local function nodedebugtextModeChange(change)
  initBDebugImpl()
  bdebugImpl.nodedebugtextModeChange(change)
end

local function skeletonModeChange(change)
  initBDebugImpl()
  bdebugImpl.skeletonModeChange(change)
end

local function colTrisModeChange(change)
  initBDebugImpl()
  bdebugImpl.colTrisModeChange(change)
end

local function cogChange(change)
  initBDebugImpl()
  bdebugImpl.cogChange(change)
end

local function resetModes()
  initBDebugImpl()
  bdebugImpl.resetModes()
end


-- Following interfaces used for setting for bdebugImpl and can be called before bdebugImpl is initalized.
-- When initialized, whatever data set before initialization is passed to it.

local function setPartsSelected(parts)
  _state.partsSelected = parts
end

-- Sets the text to display at a node using the node debug text visualization
-- "type" is the group the text belongs to
-- "nodeCID" is the id of the node at runtime
-- "text" is the text you want to display at the node
local function setNodeDebugText(type, nodeCID, text)
  -- If type doesn't exist, create it!
  local id = _state.vehicle.nodeDebugTextTypeToID[type]
  if not id then
    table.insert(_state.vehicle.nodeDebugTextModes, {name = type, data = {}})
    _state.vehicle.nodeDebugTextTypeToID[type] = #_state.vehicle.nodeDebugTextModes
    id = _state.vehicle.nodeDebugTextTypeToID[type]
  end
  local mode = _state.vehicle.nodeDebugTextModes[id]

  -- If node data doesn't exist, create it!
  if not mode.data[nodeCID] then
    mode.data[nodeCID] =
    {
      textList = {},
    }
  end

  -- Add text to list
  table.insert(
    mode.data[nodeCID].textList,
    text
  )
end

-- Removes the text displaying at a node
-- "type" is the text group
-- "nodeCID" is the id of the node at runtime
local function clearNodeDebugText(type, nodeCID)
  local id = _state.vehicle.nodeDebugTextTypeToID[type]
  if id then
    _state.vehicle.nodeDebugTextModes[id].data[nodeCID] = nil
  end
end

-- Removes a specific text group
-- "type" is the text group
local function clearTypeNodeDebugText(type)
  local id = _state.vehicle.nodeDebugTextTypeToID[type]
  if id then
    table.remove(_state.vehicle.nodeDebugTextModes, id)

    -- Subtract one from mode to keep same mode selected
    if _state.vehicle.nodeDebugTextMode >= id then
      _state.vehicle.nodeDebugTextMode = _state.vehicle.nodeDebugTextMode - 1
    end
    _state.vehicle.nodeDebugTextTypeToID[type] = nil

    -- Update type to ID lookups as the ids have been shifted down
    for i = id, #_state.vehicle.nodeDebugTextModes do
      local currType = _state.vehicle.nodeDebugTextModes[i].name
      _state.vehicle.nodeDebugTextTypeToID[currType] = _state.vehicle.nodeDebugTextTypeToID[currType] - 1
    end
  end
end

-- Removes all text groups
local function clearAllNodeDebugText()
  _state.vehicle.nodeDebugTextModes = {{name = "off"}}
  _state.vehicle.nodeDebugTextMode = 1
  table.clear(_state.vehicle.nodeDebugTextTypeToID)
end


-- Persists bdebugImpl state across vehicle reloads

local function onSerialize()
  return {
    bdebugImplState = bdebugImpl and bdebugImpl.state,
  }
end

local function onDeserialize(data)
  if data.bdebugImplState then
    initBDebugImpl(data.bdebugImplState)
  end
end

local function init()
end

M.recieveViewportSize = function() end

-- These interfaces initally nopped. Unnopped when bdebugImpl is initialized.
M.nodeCollision = nop
M.beamBroke = nop
M.debugDraw = nop
M.onPlayersChanged = nop
M.reset = nop

-- Following interfaces will initalize bdebugImpl
M.setState = setState
M.requestState = sendState
M.requestDrawnNodesGE = requestDrawnNodesGE
M.requestDrawnBeamsGE = requestDrawnBeamsGE
M.setPartsSelected = setPartsSelected
M.isEnabled = isEnabled
M.setEnabled = setEnabled
M.toggleEnabled = toggleEnabled
M.nodetextModeChange = nodetextModeChange
M.nodevisModeChange = nodevisModeChange
M.nodedebugtextModeChange = nodedebugtextModeChange
M.skeletonModeChange = skeletonModeChange
M.colTrisModeChange = colTrisModeChange
M.cogChange = cogChange
M.resetModes = resetModes

-- Following interfaces used for setting for bdebugImpl and can be called before bdebugImpl is initalized.
-- When initialized, whatever data set before initialization is passed to it.
M.setPartsSelected = setPartsSelected

M.setNodeDebugText = setNodeDebugText
M.clearNodeDebugText = clearNodeDebugText
M.clearTypeNodeDebugText = clearTypeNodeDebugText
M.clearAllNodeDebugText = clearAllNodeDebugText

-- Saves on vehicle reload if bdebugImpl was initialized in the past, so that bdebugImpl is initialized right after vehicle reload if true
M.onSerialize = onSerialize
M.onDeserialize = onDeserialize

M.init = init

return M