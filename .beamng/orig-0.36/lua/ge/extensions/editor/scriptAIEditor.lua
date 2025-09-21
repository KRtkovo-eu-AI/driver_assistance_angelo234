-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

-- Constants / typedefs.
local im = ui_imgui
local abs, min, max = math.abs, math.min, math.max
local sqrt, tan, floor = math.sqrt, math.tan, math.floor
local consts = {
  incMPS = 0.27777777777777777777777,
  minMPS = 0.1,
  maxMPS = 41.6666666666666666666667,
  stdFwd = vec3(0.0, 1.0, 0.0),
  stdUp = vec3(0.0, 0.0, 1.0)
}

local editModeName = "Edit ScriptAI Scenario"

-- State counters.
local timer = hptimer()
local uniqueId = 0
local colCtr = 0

-- Asset collections.
local sceneVehicles = {}
local trajectories = {}

-- Mouse drag/drop state.
local mState = {
  beginDragRotation = nil,
  beginDragPos = nil,
  isDragArmed = false,
  nodeSelectData = nil,
  vehSelectData = nil,
  camSelectData = nil
}

-- Window stylings.
local colors = {
  textA = ColorF(0.1, 0.1, 0.1, 1.0),
  textB = ColorI(255, 255, 255, 192),
  fField = ColorF(0.5, 0.5, 0.5, 0.3),
  nGlow = ColorF(0.1, 0.1, 0.1, 0.5),
  bGlow = ColorF(0.5, 0.5, 0.5, 0.5),
  rec = im.ImVec4(1, 0.0, 0.0, 0.85),
  lock = im.ImVec4(0.05, 0.05, 0.05, 1.0),
  wOpen = im.ImVec4(1.0, 1.0, 1.0, 0.5),
  white = im.ImVec4(1.0, 1.0, 1.0, 1.0),
  black = im.ImVec4(0.0, 0.0, 0.0, 1.0)
}

-- Main tool window state.
local toolWinData = {
  name = "scriptAIEditor",
  winSize = im.ImVec2(561, 65),
  t = im.FloatPtr(0.0),
  tStart = 0.0,
  tEnd = 0.0,
  rewJump = 0.5,
  ffwdJump = 0.5,
  isPlaying = false,
  isExecuting = false,
  windowPos = im.ImVec2(1000, 10),
  isLooping = im.BoolPtr(false),
  manualT = im.FloatPtr(30.0),
  isDispInExe = im.BoolPtr(true),
  isOverlay = im.BoolPtr(true)
}

-- Vehicles window state.
local vehWinData = {
  name = "scriptAIEditor_VehWin",
  winSize = im.ImVec2(482, 250),
  isRecording = {},
  recordMode = {},
  selectedVeh = 1
}

-- Trajectory list window state.
local trajWinData = {
  name = "scriptAIEditor_TrajListWin",
  winSize = im.ImVec2(182, 288),
  selectedTraj = nil
}

-- The three individual trajectory windows state.
local indTrajWinData = {
  ctr = 1,
  name1 = "scriptAIEditor_TrajWin1",
  idx1 = nil,
  name2 = "scriptAIEditor_TrajWin2",
  idx2 = nil,
  name3 = "scriptAIEditor_TrajWin3",
  idx3 = nil,
  winSize = im.ImVec2(342, 500)
}

-- Camera window state.
local camWinData = {
  name = "scriptAIEditor_CamWin",
  winSize = im.ImVec2(833, 182),
  nodes = {},
  spline = {},
  selectedNode = 1,
  fieldRange = im.FloatPtr(1),
  col = im.ArrayFloat(3),
  isRigidTranslation = im.BoolPtr(false),
  isDisplay = im.BoolPtr(true),
  isOnExecute = im.BoolPtr(false)
}

-- Draw window state.
local drawWinData = {
  name = "scriptAIEditor_DrawWin",
  winSize = im.ImVec2(226, 365),
  drawNodes = {},
  mode = im.IntPtr(1),
  isDrawIn = false,
  drawCol = im.ArrayFloat(3)
}

-- State for windows after onDeserialized
local winStates = {
  vehWin = {visible = false, buttonAlpha = 1},
  trajWin = {visible = false, buttonAlpha = 1},
  camWin = {visible = false, buttonAlpha = 1}
}

local _uid = 0 -- do not use ever
local function getNextUniqueId()
  _uid = _uid + 1
  return _uid
end

-- Rounding functions.
local function round1(n) return tonumber(string.format("%.1f", n)) end
local function round2(n) return tonumber(string.format("%.2f", n)) end

-- Converters for re-formatting single nodes, between ImGui pointer style and value style, for time-based and speed-based scripts.
local function ptr2ValT(d) return { x = d.x[0], y = d.y[0], z = d.z[0], t = d.t[0], isLocked = d.isLocked  } end
local function val2PtrT(d) return { x = im.FloatPtr(d.x), y = im.FloatPtr(d.y), z = im.FloatPtr(d.z), t = im.FloatPtr(d.t), isLocked = d.isLocked  } end
local function ptr2ValV(d) return { x = d.x[0], y = d.y[0], z = d.z[0], v = d.v[0], isLocked = d.isLocked  } end
local function val2PtrV(d) return { x = im.FloatPtr(d.x), y = im.FloatPtr(d.y), z = im.FloatPtr(d.z), v = im.FloatPtr(d.v), isLocked = d.isLocked  } end

-- Gets a reference to a trajectory polyline or spline (depending on which has been selected by the user).
local function getPolyRef(tr)
  if tr.isUseSpline[0] == true and tr.spline ~= nil then return tr.spline end
  return tr.polyLine
end

-- Converts an array of nodes from value style to ImGui pointer style, for time-based scripts.
local function polyVal2PtrT(d)
  local n, len = {}, #d
  for i = 1, len do
    n[i] = val2PtrT(d[i])
  end
  return n
end

-- Converts an array of nodes from ImGui pointer style to value style, for time-based scripts.
local function polyPtr2ValT(d)
  local n, len = {}, #d
  for i = 1, len do
    n[i] = ptr2ValT(d[i])
  end
  return n
end

-- Converts an array of nodes from value style to ImGui pointer style, for speed-based scripts.
local function polyVal2PtrV(d)
  local n, len = {}, #d
  for i = 1, len do
    n[i] = val2PtrV(d[i])
  end
  return n
end

-- Converts an array of nodes from ImGui pointer style to value style, for speed-based scripts.
local function polyPtr2ValV(d)
  local n, len = {}, #d
  for i = 1, len do
    n[i] = ptr2ValV(d[i])
  end
  return n
end

-- Converts the camera path nodes from ImGui pointer style to value style.
local function camPathPtr2Val()
  local nds = camWinData.nodes
  local len, out = #nds, {}
  for i = 1, len do
    local n = nds[i]
    out[i] = {
      t = n.t[0],
      x = n.x[0], y = n.y[0], z = n.z[0],
      qx = n.qx, qy = n.qy, qz = n.qz, qw = n.qw,
      smoothness = n.smoothness[0],
      movingStart = n.movingStart[0], movingEnd = n.movingEnd[0],
      isLocked = n.isLocked }
  end
  return out
end

-- Converts the camera path nodes from value style to ImGui pointer style.
local function camPathVal2Ptr(d)
  table.clear(camWinData.nodes)
  local len = #d
  for i = 1, len do
    local n = d[i]
    camWinData.nodes[i] = {
      t = im.FloatPtr(n.t),
      x = im.FloatPtr(n.x), y = im.FloatPtr(n.y), z = im.FloatPtr(n.z),
      qx = n.qx, qy = n.qy, qz = n.qz, qw = n.qw,
      smoothness = im.FloatPtr(n.smoothness),
      movingStart = im.BoolPtr(n.movingStart), movingEnd = im.BoolPtr(n.movingEnd),
      isLocked = n.isLocked }
  end
end

-- Sets the times of a given array of nodes in value style, so as to maintain a given velocity throughout the path, for time-based mode.
local function setFixedVelValT(data, vel)
  local nodes, len, velInv = {}, #data, 1.0 / vel
  for i = 1, len do
    local n1 = data[i]
    n1.t = 0.0
    if i > 1 then
      local last = i - 1
      local n0 = data[last]
      n1.t = nodes[last].t[0] + ((vec3(n1.x, n1.y, n1.z) - vec3(n0.x, n0.y, n0.z)):length() * velInv)
    end
    nodes[i] = val2PtrT(n1)
  end
  return nodes
end

-- Sets the times of a given array of nodes in ImGui pointer style, so as to maintain a given velocity throughout the path.
local function setFixedVelPtr(d, vel)
  local len, velInv = #d, 1.0 / vel
  for i = 2, len do
    local n0, n1 = d[i - 1], d[i]
    local timeStamp = n0.t[0] + ((vec3(n1.x[0], n1.y[0], n1.z[0]) - vec3(n0.x[0], n0.y[0], n0.z[0])):length() * velInv)
    n1.t = im.FloatPtr(timeStamp)
  end
  return d
end

-- Converts a polyline to nodes, used during execution or export.
local function buildNodes(poly, tr)
  local nodes, len = {}, #poly
  if tr.isTimeBased == true then
    for i = 1, len do
      local n = poly[i]
      nodes[i] = { x = n.x[0], y = n.y[0], z = n.z[0], t = n.t[0] }
    end
  else
    for i = 1, len do
      local n = poly[i]
      nodes[i] = { x = n.x[0], y = n.y[0], z = n.z[0], v = n.v[0] }
    end
  end

  if not nodes[1] then
    nodes[1] = { x = 0, y = 0, z = 0, t = 0, v = 0 }
  end
  nodes[1].dir, nodes[1].up = vec3(consts.stdFwd), vec3(consts.stdUp)                  -- The first node is special, since it contains the dir and up vectors.
  local firstPos = vec3(nodes[1].x, nodes[1].y, nodes[1].z)
  for i, n in ipairs(nodes) do
    if i > 1 then
      local pos = vec3(n.x, n.y, n.z)
      local dirVec = pos - firstPos
      if dirVec:squaredLength() >= 1 then                                              -- Threshold of 1 m, to prevent tiny movements from causing a wrong direction.
        dirVec:normalize()
        nodes[1].dir:set(dirVec)
        nodes[1].up:set(map.surfaceNormal(pos, 1))
        break
      end
    end
  end

  return nodes
end

-- Deep copies an array of nodes (without time stamps).
local function copy(d)
  local c, len = {}, #d
  for i = 1, len do
    local n = d[i]
    c[i] = { x = n.x, y = n.y, z = n.z, isLocked = false }
  end
  return c
end

-- Fetches a new default color by cycling through a simple palette.
local function getNextColor()
  local col, v = im.ArrayFloat(3), colCtr % 6
  if v == 0 then
    col[0], col[1], col[2] = im.Float(1.0), im.Float(0.0), im.Float(0.0)
  elseif v == 1 then
    col[0], col[1], col[2] = im.Float(0.0), im.Float(0.0), im.Float(1.0)
  elseif v == 2 then
    col[0], col[1], col[2] = im.Float(0.0), im.Float(1.0), im.Float(0.0)
  elseif v == 3 then
    col[0], col[1], col[2] = im.Float(1.0), im.Float(0.0), im.Float(1.0)
  elseif v == 4 then
    col[0], col[1], col[2] = im.Float(1.0), im.Float(1.0), im.Float(0.0)
  else
    col[0], col[1], col[2] = im.Float(0.0), im.Float(1.0), im.Float(1.0)
  end
  colCtr = colCtr + 1
  return col
end

-- Assigns the trajectory with the given index to one of the individual trajectory display windows, cycling them. If already displaying it, close it.
local function assignTrajWin(idx)
  local itwd = indTrajWinData
  if itwd.idx1 == idx then                                                        -- If we are displaying the given trajectory, then close its window and leave.
    editor.hideWindow(itwd.name1)
    itwd.idx1, itwd.ctr = nil, 1
    return
  elseif itwd.idx2 == idx then
    editor.hideWindow(itwd.name2)
    itwd.idx2, itwd.ctr = nil, 2
    return
  elseif itwd.idx3 == idx then
    editor.hideWindow(itwd.name3)
    itwd.idx3, itwd.ctr = nil, 3
    return
  end
  if itwd.ctr == 1 then                                                           -- Show the trajectory in the next available window, and cycle the counter.
    itwd.idx1, itwd.ctr = idx, 2
    editor.showWindow(itwd.name1)
  elseif itwd.ctr == 2 then
    itwd.idx2, itwd.ctr = idx, 3
    editor.showWindow(itwd.name2)
  else
    itwd.idx3, itwd.ctr = idx, 1
    editor.showWindow(itwd.name3)
  end
end

-- Gets the color for the 'show trajectory' button. Bright if trajectory is currently displayed, otherwise duller.
local function getTrajButtonCol(idx)
  local itwd = indTrajWinData
  if itwd.idx1 == idx or itwd.idx2 == idx or itwd.idx3 == idx then return colors.wOpen end
  return colors.white
end

-- Converts a single trajectory which contains many ImGui pointers, into an array containing only value data.
local function ptr2ValTraj(tr)
  local p, s = nil, nil
  if tr.isTimeBased == true then
    p = polyPtr2ValT(tr.polyLine)
    if tr.spline ~= nil then
      s = polyPtr2ValT(tr.spline)
    end
  else
    p = polyPtr2ValV(tr.polyLine)
    if tr.spline ~= nil then
      s = polyPtr2ValV(tr.spline)
    end
  end
  return {
    vehicle = tr.vehicle, vid = tr.vid, jBeam = tr.jBeam,
    isTimeBased = tr.isTimeBased,
    polyLine = p, spline = s,
    isExternalForce = tr.isExternalForce[0],
    isHoldVelocity = tr.isHoldVelocity[0],
    isDisplay = tr.isDisplay[0], isMarkNodes = tr.isMarkNodes[0], isMarkVelocities = tr.isMarkVelocities[0],
    colR = tr.col[0], colG = tr.col[1], colB = tr.col[2],
    selectedNode = tr.selectedNode,
    fieldRange = tr.fieldRange[0],
    isUseRigidTranslation = tr.isUseRigidTranslation[0],
    splineSpacing = tr.splineSpacing[0], isUseSpline = tr.isUseSpline[0],
    boxPos = tr.boxPos,
    inputVelocity = tr.inputVelocity[0],
    vModeTStart = tr.vModeTStart[0], vModeTEnd = tr.vModeTEnd[0] }
end

-- Serializes the full trajectory data table.
local function serializeTrajData()
  local out = {}
  for id, tr in pairs(trajectories) do
    out[id] = ptr2ValTraj(tr)
  end
  return out
end

-- Converts a single trajectory which contains all value data, into an array which contains some pointer data.
local function val2PtrTraj(tr)
  local p, s = nil, nil
  if tr.isTimeBased == true then
    p = polyVal2PtrT(tr.polyLine)
    if tr.spline ~= nil then
      s = polyVal2PtrT(tr.spline)
    end
  else
    p = polyVal2PtrV(tr.polyLine)
    if tr.spline ~= nil then
      s = polyVal2PtrV(tr.spline)
    end
  end
  local col = im.ArrayFloat(3)
  col[0], col[1], col[2]  = tr.colR, tr.colG, tr.colB
  return {
    vehicle = tr.vehicle, vid = tr.vid, jBeam = tr.jBeam,
    isTimeBased = tr.isTimeBased,
    polyLine = p, spline = s,
    isExternalForce = im.BoolPtr(tr.isExternalForce),
    isHoldVelocity = im.BoolPtr(tr.isHoldVelocity),
    isMarkNodes = im.BoolPtr(tr.isMarkNodes), isMarkVelocities = im.BoolPtr(tr.isMarkVelocities), isDisplay = im.BoolPtr(tr.isDisplay),
    col = col,
    selectedNode = tr.selectedNode,
    fieldRange = im.FloatPtr(tr.fieldRange),
    isUseRigidTranslation = im.BoolPtr(tr.isUseRigidTranslation),
    splineSpacing = im.IntPtr(tr.splineSpacing), isUseSpline = im.BoolPtr(tr.isUseSpline),
    boxPos = tr.boxPos,
    inputVelocity = im.FloatPtr(tr.inputVelocity),
    vModeTStart = im.FloatPtr(tr.vModeTStart), vModeTEnd = im.FloatPtr(tr.vModeTEnd) }
  end

-- Deserializes the full trajectory data table.
local function deserializeTrajData(data)
  table.clear(trajectories)
  for id, tr in pairs(data) do
    trajectories[id] = val2PtrTraj(tr)
  end
end

-- Serializes the camera trajectory data table.
local function serializeCamData()
  local cwd = camWinData
  local nodes = cwd.nodes
  local len, nodesVal = #nodes, {}
  for i = 1, len do
    local n = nodes[i]
    nodesVal[i] = {
      x = n.x[0], y = n.y[0], z = n.z[0],
      qx = n.qx, qy = n.qy, qz = n.qz, qw = n.qw,
      t = n.t[0],
      smoothness = n.smoothness[0],
      movingStart = n.movingStart[0], movingEnd = n.movingEnd[0],
      isLocked = n.isLocked }
  end
  return {
    name = "scriptAIEditor_CamWin",
    nodes = nodesVal,
    selectedNode = cwd.selectedNode,
    colR = cwd.col[0], colG = cwd.col[1], colB = cwd.col[2],
    isDisplay = cwd.isDisplay[0],
    isOnExecute = cwd.isOnExecute[0] }
end

-- Deserializes the camera trajectory data table.
local function deserializeCamData(data)
  local cwd = camWinData
  table.clear(cwd.nodes)
  local len = #data.nodes
  for i = 1, len do
    local n = data.nodes[i]
    cwd.nodes[i] = {
      x = im.FloatPtr(n.x), y = im.FloatPtr(n.y), z = im.FloatPtr(n.z),
      qx = n.qx, qy = n.qy, qz = n.qz, qw = n.qw,
      t = im.FloatPtr(n.t),
      smoothness = im.FloatPtr(n.smoothness),
      movingStart = im.BoolPtr(n.movingStart), movingEnd = im.BoolPtr(n.movingEnd),
      isLocked = n.isLocked }
  end
  local col = im.ArrayFloat(3)
  col[0], col[1], col[2]  = data.colR, data.colG, data.colB
  cwd.name = data.name
  cwd.selectedNode = data.selectedNode
  cwd.col = col
  cwd.isDisplay = im.BoolPtr(data.isDisplay)
  cwd.isOnExecute = im.BoolPtr(data.isOnExecute)
end

-- Performs an 'undo' operation on a polyline.
local function nodesUndo(data)
  local tr, nodes = trajectories[data.tIdx], nil
  if data.isSpline == true then
    nodes = tr.spline
  else
    nodes = tr.polyLine
  end
  table.clear(nodes)
  local len = #data.old
  if tr.isTimeBased == true then
    for i = 1, len do
      local d = data.old[i]
      nodes[i] = { x = im.FloatPtr(d.x), y = im.FloatPtr(d.y), z = im.FloatPtr(d.z), t = im.FloatPtr(d.t), isLocked = d.isLocked }
    end
  else
    for i = 1, len do
      local d = data.old[i]
      nodes[i] = { x = im.FloatPtr(d.x), y = im.FloatPtr(d.y), z = im.FloatPtr(d.z), v = im.FloatPtr(d.v), isLocked = d.isLocked }
    end
  end
  local sn = trajectories[data.tIdx].selectedNode
  if sn > #nodes then
    sn = 1
  end
end

-- Performs a 'redo' operation on a polyline.
local function nodesRedo(data)
  local tr, nodes = trajectories[data.tIdx], nil
  if data.isSpline == true then
    nodes = tr.spline
  else
    nodes = tr.polyLine
  end
  table.clear(nodes)
  local len = #data.new
  if tr.isTimeBased == true then
    for i = 1, len do
      local d = data.new[i]
      nodes[i] = { x = im.FloatPtr(d.x), y = im.FloatPtr(d.y), z = im.FloatPtr(d.z), t = im.FloatPtr(d.t), isLocked = d.isLocked }
    end
  else
    for i = 1, len do
      local d = data.new[i]
      nodes[i] = { x = im.FloatPtr(d.x), y = im.FloatPtr(d.y), z = im.FloatPtr(d.z), v = im.FloatPtr(d.v), isLocked = d.isLocked }
    end
  end
  local sn = trajectories[data.tIdx].selectedNode
  if sn > #nodes then
    sn = 1
  end
end

-- Performs undo/redo operations wrt the full table of trajectories, or individual trajectories.
local function trajWinUndo(data) deserializeTrajData(data.old) end
local function trajWinRedo(data) deserializeTrajData(data.new) end
local function indTrajWinUndo(data) trajectories[data.tIdx] = val2PtrTraj(data.old) end
local function indTrajWinRedo(data) trajectories[data.tIdx] = val2PtrTraj(data.new) end

-- Performs undo/redo operations wrt the camera data table or only the camera nodes array.
local function camWinUndo(data) deserializeCamData(data.old) end
local function camWinRedo(data) deserializeCamData(data.new) end
local function camNodesUndo(data) camPathVal2Ptr(data.old) end
local function camNodesRedo(data) camPathVal2Ptr(data.new) end

-- Performs undo/redo operations wrt the polyline which is currently being drawn.
local function drawUndo(data) drawWinData.drawNodes = data.old end
local function drawRedo(data) drawWinData.drawNodes = data.new end

-- Performs an undo operation on a gizmo translate operation.
local function gizmoPosUndo(data)
  local n, old = camWinData.nodes[data.idx], data.old
  n.x, n.y, n.z = im.FloatPtr(old.x), im.FloatPtr(old.y), im.FloatPtr(old.z)
end

-- Performs a redo operation on a gizmo translate operation.
local function gizmoPosRedo(data)
  local n, new = camWinData.nodes[data.idx], data.new
  n.x, n.y, n.z = im.FloatPtr(new.x), im.FloatPtr(new.y), im.FloatPtr(new.z)
end

-- Performs an undo operation on a gizmo rotate operation.
local function gizmoRotUndo(data)
  local n, old = camWinData.nodes[data.idx], data.old
  n.qx, n.qy, n.qz, n.qw = old.x, old.y, old.z, old.w
end

-- Performs a redo operation on a gizmo rotate operation.
local function gizmoRotRedo(data)
  local n, new = camWinData.nodes[data.idx], data.new
  n.qx, n.qy, n.qz, n.qw = new.x, new.y, new.z, new.w
end

-- Callback for begin axis gizmo dragging.
local function gizmoBeginDrag()
  local n = camWinData.nodes[camWinData.selectedNode]
  mState.beginDragPos, mState.beginDragRotation = vec3(n.x[0], n.y[0], n.z[0]), quat(n.qx, n.qy, n.qz, n.qw)
end

-- Callback for end axis gizmo dragging.
local function gizmoEndDrag()
  local cwd = camWinData
  local idx = cwd.selectedNode
  local n = cwd.nodes[idx]
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then
    local data = { idx = idx, old = mState.beginDragPos, new = vec3(n.x[0], n.y[0], n.z[0]) }
    editor.history:commitAction("Translate Camera Path Node", data, gizmoPosUndo, gizmoPosRedo)
  elseif editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then
    local data = { idx = idx, old = mState.beginDragRotation, new = quat(n.qx, n.qy, n.qz, n.qw) }
    editor.history:commitAction("Rotate Camera Path Node", data, gizmoRotUndo, gizmoRotRedo)
  end
end

-- Callback for continuing axis gizmo dragging.
local function gizmoDragging()
  local cwd = camWinData
  local idx = cwd.selectedNode
  local n = cwd.nodes[idx]
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then             -- Handle dragging on the translation gizmo.
    local p = editor.getAxisGizmoTransform():getColumn(3)
    n.x, n.y, n.z = im.FloatPtr(p.x), im.FloatPtr(p.y), im.FloatPtr(p.z)
    return
  end
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then                -- Handle dragging on the rotational gizmo.
    local rotMat = editor.getAxisGizmoTransform()
    local q2 = QuatF(0, 0, 0, 1)
    q2:setFromMatrix(rotMat)
    local q = mState.beginDragRotation * quat(q2.x, q2.y, q2.z, q2.w)
    n.qx, n.qy, n.qz, n.qw = q.x, q.y, q.z, q.w
  end
end

-- Compute the minimum R^3 distance between a given point and the line segment between two other given points.
local function minDistPointLineSeg(v, a, b)
  local ab, av  = b - a, v - a
  if av:dot(ab) <= 0.0 then
    return av:length()
  end
  local bv  = v - b
  if bv:dot(ab) >= 0.0 then
    return bv:length()
  end
  return (ab:cross(av)):length() / ab:length()
end

-- Computes the two bounding node indices for the current scenario time.
local function getBounds(d)
  local tNow = toolWinData.t[0]
  local l, u, lClosest, uClosest, len = nil, nil, 1e30, 1e30, #d
  for i = 1, len do
    local t = d[i].t[0]
    if t <= tNow then
      local dt = tNow - t
      if dt < lClosest then                                                       -- If we find the closest lower bound so far, store it.
        lClosest, l = dt, i
      end
    end
    if t >= tNow then
      local dt = t - tNow
      if dt < uClosest then                                                       -- If we find a closest upper bound so far, store it.
        uClosest, u = dt, i
      end
    end
  end
  if l == nil then                                                                -- If a lower bound cannot be found, default it to the first node in the trajectory.
    l = 1
  end
  if u == nil then                                                                -- If an upper bound cannot be found, default it to the last node in the trajectory.
    u = len
  end
  return l, u
end

-- Computes the lower and upper indices at which the first lock is found, inclusive of the two given bounding nodes.
local function getLockBounds(poly, lower, upper)
  local lLock = nil
  for i = lower, 1, -1 do                                                         -- Iterate down through the trajectory to find the first lock in the backwards direction.
    if poly[i].isLocked == true then
      lLock = i
      break
    end
  end
  local uLock, len = nil, #poly
  for i = upper, len do                                                           -- Iterate up through the trajectory to find the first lock in the forward direction.
    if poly[i].isLocked == true then
      uLock = i
      break
    end
  end
  return lLock, uLock
end

-- Computes the average velocity between two nodes on a trajectory, in m/s.
local function getVel(poly, a, b)
  local n1, n2 = poly[a], poly[b]
  local p1, p2 = vec3(n1.x[0], n1.y[0], n1.z[0]), vec3(n2.x[0], n2.y[0], n2.z[0])
  return (p2 - p1):length() / (n2.t[0] - n1.t[0])
end

-- Computes the two points on a trajectory which bound the current time position, and the relevant parameter in [0, 1].
local function lerpTraj(tr)
  local poly = getPolyRef(tr)
  local lower, upper = getBounds(poly, false)
  local len = #poly
  if lower == len then                                                            -- Case #1: The current time is after the last trajectory node.
    local n = poly[len]
    local p1 = vec3(n.x[0], n.y[0], n.z[0])
    return p1, p1, 0.0
  elseif upper == 1 then                                                          -- Case #2: The current time is before the first trajectory node.
    local n = poly[1]
    local p1 = vec3(n.x[0], n.y[0], n.z[0])
    return p1, p1, 0.0
  else                                                                            -- Case #3 [typical]: The current time is between two trajectory nodes.
    local n1, n2 = poly[lower], poly[upper]
    local t1, t2 = n1.t[0], n2.t[0]
    local a = (toolWinData.t[0] - t1) / (t2 - t1)
    return vec3(n1.x[0], n1.y[0], n1.z[0]), vec3(n2.x[0], n2.y[0], n2.z[0]), a
  end
end

-- Draws an eight-point oriented bounding box at the given position (front position), with the given dimensions.
local function drawVehBox(d, isHighlighted)
  local pos, fwd, w, l = d.pos, d.dir, d.width, d.length
  local right = fwd:cross(consts.stdUp)
  right:normalize()
  local up = -fwd:cross(right)
  local wHalf, lHalf = w * 0.5, l * 0.5
  local whr, whf = wHalf * right, lHalf * fwd
  pos = pos - fwd * lHalf
  local c1, c2, c3, c4 = pos - whr + whf, pos + whr + whf, pos - whr - whf, pos + whr - whf
  local up2 = 2.0 * up
  local c5, c6, c7, c8 = c1 + up2, c2 + up2, c3 + up2, c4 + up2
  local thickness, col = 4, colors.textA
  if isHighlighted == true then
    thickness, col = 10, colors.bGlow
  end
  debugDrawer:drawSphere(pos, 0.1, colors.textA)                                  -- A small sphere at the center of the box.
  debugDrawer:drawLineInstance(c1, c2, thickness, col)                            -- The bottom four lines.
  debugDrawer:drawLineInstance(c3, c4, thickness, col)
  debugDrawer:drawLineInstance(c1, c3, thickness, col)
  debugDrawer:drawLineInstance(c2, c4, thickness, col)
  debugDrawer:drawLineInstance(c5, c6, thickness, col)                            -- The top four lines.
  debugDrawer:drawLineInstance(c7, c8, thickness, col)
  debugDrawer:drawLineInstance(c5, c7, thickness, col)
  debugDrawer:drawLineInstance(c6, c8, thickness, col)
  debugDrawer:drawLineInstance(c1, c5, thickness, col)                            -- The four vertical lines.
  debugDrawer:drawLineInstance(c2, c6, thickness, col)
  debugDrawer:drawLineInstance(c3, c7, thickness, col)
  debugDrawer:drawLineInstance(c4, c8, thickness, col)
end

-- Draws a camera frame box at the current time on the camera trajectory.
local function drawCamBox(p, q, isHighlighted)
  local fwd, up = q:toDirUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  local wHalf, lHalf, hHalf = 0.5, 0.5, 0.5
  local whr, whf, whu = wHalf * right, lHalf * fwd, hHalf * up
  local c1, c2, c3, c4 = p - whr + whf - whu, p + whr + whf - whu, p - whr - fwd - whu, p + whr - fwd - whu
  local c5, c6, c7, c8 = c1 + up, c2 + up, c3 + up, c4 + up
  local e1, e2 = p + fwd - right - whu, p + fwd + right - whu
  local e3, e4 = e1 + up, e2 + up
  local thickness, col = 4, colors.textA
  if isHighlighted == true then
    thickness, col = 10, colors.bGlow
  end
  debugDrawer:drawSphere(p, 0.1, colors.textA)                                    -- A small sphere at the center of the box.
  debugDrawer:drawArrow(p, p + fwd * 2, ColorI(0, 0, 0, 255), true)
  debugDrawer:drawLineInstance(c1, c2, thickness, col)                            -- The bottom four lines.
  debugDrawer:drawLineInstance(c3, c4, thickness, col)
  debugDrawer:drawLineInstance(c1, c3, thickness, col)
  debugDrawer:drawLineInstance(c2, c4, thickness, col)
  debugDrawer:drawLineInstance(c5, c6, thickness, col)                            -- The top four lines.
  debugDrawer:drawLineInstance(c7, c8, thickness, col)
  debugDrawer:drawLineInstance(c5, c7, thickness, col)
  debugDrawer:drawLineInstance(c6, c8, thickness, col)
  debugDrawer:drawLineInstance(c1, c5, thickness, col)                            -- The four vertical lines.
  debugDrawer:drawLineInstance(c2, c6, thickness, col)
  debugDrawer:drawLineInstance(c3, c7, thickness, col)
  debugDrawer:drawLineInstance(c4, c8, thickness, col)
  debugDrawer:drawLineInstance(e1, e2, thickness, col)                            -- The aperture end lines
  debugDrawer:drawLineInstance(e3, e4, thickness, col)
  debugDrawer:drawLineInstance(e1, e3, thickness, col)
  debugDrawer:drawLineInstance(e2, e4, thickness, col)
  debugDrawer:drawLineInstance(c1, e1, thickness, col)                            -- The box-to-aperture lines.
  debugDrawer:drawLineInstance(c2, e2, thickness, col)
  debugDrawer:drawLineInstance(c5, e3, thickness, col)
  debugDrawer:drawLineInstance(c6, e4, thickness, col)
end

-- Finds the corresponding data (display string and jBeam) of the vehicle with the given id. If nothing found, returns empty defaults.
local function findVehDataByVid(vid)
  local len = #sceneVehicles
  for i = 1, len do
    local v = sceneVehicles[i]
    if v.vid == vid then
      return v.string, v.jBeam
    end
  end
  return "", nil
end

-- Creates a trajectory from some given polyLine data, and an optional vehicle Id.
local function createTrajectory(d, vehId, color, externalForce, isTimeBased)
  local nodes = {}
  if isTimeBased == true then                                                     -- Case #V: We have a speed-based script with speed values supplied.
    nodes = polyVal2PtrT(d)
  else                                                                            -- Case #T: We have a time-based script with time values supplied.
    nodes = polyVal2PtrV(d)
  end
  local len = #nodes
  for i = 1, len do                                                               -- Ensure the locking properties is added, and defaulted to false everywhere.
    nodes[i].isLocked = false
  end
  local ef = im.BoolPtr(true)                                                     -- Set the external force flag (for AI assistance).
  if externalForce ~= nil then
    ef = im.BoolPtr(externalForce)
  end
  local attachingVehicle, jBeam = "", nil                                         -- Attempt to link the new trajectory with a vehicle (used when data was recorded).
  if vehId ~= nil then
    attachingVehicle, jBeam = findVehDataByVid(vehId)
  end
  local c = nil                                                                   -- Get the color. If not provided, choose a new color from a small cyclic palette.
  if color ~= nil then
    c = im.ArrayFloat(3)
    c[0], c[1], c[2] = im.Float(color[0]), im.Float(color[1]), im.Float(color[2])
  else
    c = getNextColor()
  end
  return {
    vehicle = attachingVehicle, vid = vehId, jBeam = jBeam,
    isTimeBased = isTimeBased,
    polyLine = nodes, spline = nil,
    isUseSpline = im.BoolPtr(false),
    isExternalForce = ef,
    isDisplay = im.BoolPtr(true), isMarkNodes = im.BoolPtr(false),
    isMarkVelocities = im.BoolPtr(false),
    isHoldVelocity = im.BoolPtr(true),
    col = c,
    selectedNode = 1,
    fieldRange = im.FloatPtr(0.5),
    isUseRigidTranslation = im.BoolPtr(false),
    splineSpacing = im.IntPtr(5),
    boxPos = nil,
    inputVelocity = im.FloatPtr(30.0),
    vModeTStart = im.FloatPtr(0.0), vModeTEnd = im.FloatPtr(60.0) }
end

-- Adds a new node to a trajectory.
local function addNode(polyLine, pos, isTimeBased)
  local newX, newY, newZ, len = nil, nil, nil, #polyLine
  if isTimeBased == true then
    local newT = nil
    if pos < len then
      local s1, s2 = polyLine[pos], polyLine[pos + 1]
      newT, newX, newY, newZ = (s1.t[0] + s2.t[0]) * 0.5, (s1.x[0] + s2.x[0]) * 0.5, (s1.y[0] + s2.y[0]) * 0.5, (s1.z[0] + s2.z[0]) * 0.5
    else
      local s1, s2 = polyLine[pos - 1], polyLine[pos]
      newT, newX, newY, newZ = (2.0 * s2.t[0] - s1.t[0]), (2.0 * s2.x[0] - s1.x[0]), (2.0 * s2.y[0] - s1.y[0]), (2.0 * s2.z[0] - s1.z[0])
    end
    table.insert(polyLine, pos + 1, { t = im.FloatPtr(newT), x = im.FloatPtr(newX), y = im.FloatPtr(newY), z = im.FloatPtr(newZ), isLocked = false })
  else
    local newV = nil
    if pos < len then
      local s1, s2 = polyLine[pos], polyLine[pos + 1]
      newV, newX, newY, newZ = (s1.v[0] + s2.v[0]) * 0.5, (s1.x[0] + s2.x[0]) * 0.5, (s1.y[0] + s2.y[0]) * 0.5, (s1.z[0] + s2.z[0]) * 0.5
    else
      local s1, s2 = polyLine[pos - 1], polyLine[pos]
      newV, newX, newY, newZ = (2.0 * s2.v[0] - s1.v[0]), (2.0 * s2.x[0] - s1.x[0]), (2.0 * s2.y[0] - s1.y[0]), (2.0 * s2.z[0] - s1.z[0])
    end
    table.insert(polyLine, pos + 1, { v = im.FloatPtr(newV), x = im.FloatPtr(newX), y = im.FloatPtr(newY), z = im.FloatPtr(newZ), isLocked = false })
  end
end

-- Removes the vehicle with the given string from all trajectories, so it can be attached only to this one.
local function detachTrajectory(str)
  for k, tr in pairs(trajectories) do
    if tr.vehicle == str then
      tr.vehicle, tr.vid, tr.jBeam = "", nil, nil
    end
  end
end

-- Executes all the vehicle trajectories (and camera trajectory, if used) in sync.
local function execute()
  local polyLinesToExecute = {}                                                   -- First, format all the trajectories so they can be run by the scriptAI backend.
  for k, tr in pairs(trajectories) do
    if tr.vehicle ~= "" and tr.vid ~= nil then
      local poly = getPolyRef(tr)
      local vid = tr.vid                                                          -- Compute the executable script.
      local polyLine = {}
      local nodes = buildNodes(poly, tr)
      polyLine.path = nodes
      polyLinesToExecute[vid] = polyLine
      polyLine.externalForce = tr.isExternalForce[0]                              -- Add the external force (AI assist) to the top level of the script (outside 'path').
      if tr.isTimeBased == true and poly[1].t[0] > 0.0 then
        polyLine.startDelay = poly[1].t[0]
        local pathLen = #polyLine.path
        for i = 1, pathLen do
          polyLine.path[i].t = polyLine.path[i].t - polyLine.startDelay
        end
      end
    end
  end
  local cwd = camWinData                                                          -- Convert camera trajectory to a 'path' so it can be also executed, if used.
  local numNodes = #cwd.nodes
  if cwd.isOnExecute[0] == true and numNodes > 1 then
    local spline = cwd.nodes
    local path = {
      dirty = true,
      manualFov = true,
      name = "scriptAIEditor_camPath",
      markers = {} }
    for i = 1, numNodes do                                                        -- Format the camera trajectory into the 'path' format, ready for execution.
      local n = spline[i]
      path.markers[i] = {
        pos = vec3(n.x[0], n.y[0], n.z[0]), rot = quat(n.qx, n.qy, n.qz, n.qw),
        time = n.t[0],
        bullettime = 1,
        fov = 65,
        movingStart = true, movingEnd = true,
        positionSmooth = n.smoothness[0],
        trackPosition = false }
    end
    if path.markers[1].time > 0.0 then                                            -- If there is no path node at t=0.0, add a duplicate there of the first path node.
      local n = path.markers[1]
      local nStart = {
        pos = n.pos, rot = n.rot,
        time = 0.0,
        bullettime = 1,
        fov = 65,
        movingStart = true, movingEnd = true,
        positionSmooth = n.positionSmooth,
        trackPosition = false }
      table.insert(path.markers, 1, nStart)
    end
    local last = #path.markers
    if path.markers[last].time < toolWinData.tEnd then                            -- If there is no path node at t=end, add a duplicate there of the last path node.
      local n = path.markers[last]
      path.markers[last + 1] = {
        pos = n.pos, rot = n.rot,
        time = toolWinData.tEnd,
        bullettime = 1,
        fov = 65,
        movingStart = true, movingEnd = true,
        positionSmooth = n.positionSmooth,
        trackPosition = false }
    end
    core_paths.playPath(path)                                                     -- Execute the camera path.
  end
  for vid, p in pairs(polyLinesToExecute) do                                      -- Execute all the vehicle trajectories using the scriptAI backend.
    scenetree.findObject(vid):queueLuaCommand('ai.startFollowing(' .. serialize(p) .. ')')
  end
end

-- Stops executing all currently-running polyLines and returns to main state of editor.
local function stopExecute()
  for k, tr in pairs(trajectories) do                                             -- Stop executing all the vehicle scripts.
    local vid = tr.vid
    if vid ~= nil then
      scenetree.findObject(vid):queueLuaCommand('ai.stopFollowing()')
    end
  end
  local cwd = camWinData                                                          -- Stop executing the camera path, if it is being executed.
  if cwd.isOnExecute[0] == true and #cwd.nodes > 1 then
    core_paths.stopCurrentPath()
  end
end

-- Moves the camera to a top-down view of the given trajectory, such that all nodes are visible on screen.
local function moveCam2Traj(p)
  local xMin, xMax, yMin, yMax, zMax, len = 1e24, -1e24, 1e24, -1e24, -1e24, #p
  for i = 1, len do                                                               -- Compute the 2D axis-aligned bounding box of the given trajectory.
    local n = p[i]
    local nx, ny, nz = n.x[0], n.y[0], n.z[0]
    xMin, xMax, yMin, yMax, zMax = min(xMin, nx), max(xMax, nx), min(yMin, ny), max(yMax, ny), max(zMax, nz)
  end
  if xMax - xMin < 0.1 and yMax - yMin < 0.1 then return end                      -- If path is tiny (eg just a stationary vehicle), do nothing.
  local midX, midY = (xMin + xMax) * 0.5, (yMin + yMax) * 0.5                     -- Midpoint of trajectory patch.
  local groundDist = (vec3(midX, midY, 0.0) - vec3(xMax, yMax, 0.0)):length()     -- The largest distance from the center of the box to the outside.
  local halfFov = core_camera.getFovRad() * 0.5                                   -- Half the camera field of view (in radians).
  local height = groundDist / tan(halfFov) + zMax + 5.0                           -- The height that the camera should be to fit all the trajectory in view.
  local rot = quatFromDir(vec3(0, 0, -1))
  commands.setFreeCamera()
  core_camera.setPosRot(0, midX, midY, height, rot.x, rot.y, rot.z, rot.w)
end

-- Save all the session data to file, using the file dialog.
local function save(d)
  extensions.editor_fileDialog.saveFile(
    function(data)
      local vehicles, len = {}, #sceneVehicles
      for i = 1, len do
        local veh = sceneVehicles[i]
        vehicles[veh.vid] = { jBeam = veh.jBeam, config = veh.config }
      end
      local encodedData = { data = lpack.encode({
        trajectories = serializeTrajData(),
        camData = serializeCamData(),
        vehicles = vehicles,
        t = toolWinData.t[0],
        selectedVeh = vehWinData.selectedVeh,
        uniqueTrajectoryId = uniqueId,
        selectedTraj = trajWinData.selectedTraj,
        isVehWinVisible = editor.isWindowVisible(vehWinData.name),
        isTrajWinVisible = editor.isWindowVisible(trajWinData.name),
        isCamWinVisible = editor.isWindowVisible(camWinData.name) })}
      jsonWriteFile(data.filepath, encodedData, true)
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Load a previously-saved session data instance from file, using the file dialog.
local function load(d)
  extensions.editor_fileDialog.openFile(
    function(data)
      local loadedJson = jsonReadFile(data.filepath)
      local data = lpack.decode(loadedJson.data)
      deserializeTrajData(data.trajectories)
      deserializeCamData(data.camData)
      toolWinData.t = im.FloatPtr(data.t)
      vehWinData.selectedVeh = data.selectedVeh
      uniqueId = data.uniqueTrajectoryId

      if data.isVehWinVisible then editor.showWindow(vehWinData) end
      if data.isTrajWinVisible then editor.showWindow(trajWinData) end
      if data.isCamWinVisible then editor.showWindow(camWinData) end

      core_vehicles.removeAll()                                                 -- Remove any currently-spawned cars.
      local vehicles, numTrajectories, numVehicles = data.vehicles, 0, 0        -- Compute valid starting positions for all the vehicles which are to be spawned.
      for k, tr in pairs(trajectories) do
        numTrajectories = numTrajectories + 1
      end
      for k, tr in pairs(vehicles) do
        numVehicles = numVehicles + 1
      end
      local spawnPositions = {}
      if numTrajectories > 0 then
        local ctr = 1
        for k, tr in pairs(trajectories) do
          local polyLine = tr.polyLine
          local lenPoly = #polyLine
          for i = 1, lenPoly do
            local n = polyLine[i]
            local p = vec3(n.x[0], n.y[0], n.z[0])
            local isSufficientlyDistant = true
            local lenSP = #spawnPositions
            for j = 1, lenSP do
              local d = (p - spawnPositions[j]):squaredLength()
              if d < 36.0 then
                isSufficientlyDistant = false
                break
              end
            end
            if isSufficientlyDistant == true then
              spawnPositions[ctr] = p
              ctr = ctr + 1
              if ctr > numVehicles then
                break
              end
            end
          end
          if ctr > numVehicles then
            break
          end
        end
        if #spawnPositions < numVehicles then
          local ctr = #spawnPositions
          for i = 1, #spawnPositions do
            local pOld = spawnPositions[i]
            spawnPositions[ctr] = vec3(pOld.x + 15, pOld.y + 15, pOld.z)
            ctr = ctr + 1
            if ctr > numVehicles then
              break
            end
          end
        end
      end
      local ctr = 1                                                             -- Spawn all the vehicles which are required.
      for k, data in pairs(vehicles) do
        local _ = core_vehicles.spawnNewVehicle(data.jBeam, { pos = spawnPositions[ctr], config = data.config })
        ctr = ctr + 1
      end
      local alreadyAttachedVehicles = {}
      for id, tr in pairs(trajectories) do
        local vehicle = tr.vehicle
        if vehicle ~= nil and vehicle ~= "" then
          local didFindVehicleInSimulation = false                              -- Search through the spawned vehicles and find a suitable vehicle for this trajectory.
          for vid, veh in activeVehiclesIterator() do
            if alreadyAttachedVehicles[vid] == nil and tr.jBeam == veh.JBeam then
              tr.vehicle = tostring(vid .. ": " .. veh:getName() .. " - " .. veh.JBeam)
              tr.vid = vid
              tr.jBeam = veh.JBeam
              alreadyAttachedVehicles[vid] = true
              didFindVehicleInSimulation = true
            end
          end
          if didFindVehicleInSimulation == false then                           -- If no vehicle found in currently-spawned list, set the trajectory vehicle parameters to empty.
            tr.vehicle, tr.vid, tr.jBeam = "", nil, nil
          end
        end
      end
      indTrajWinData.idx1, indTrajWinData.idx2, indTrajWinData.idx3 = nil, nil, nil -- Finally, reset the individual trajectory window references.
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Import a single trajectory from file, using the file dialog.
local function import()
  extensions.editor_fileDialog.openFile(
    function(data)
      local importedData = jsonReadFile(data.filepath)
      local tIdx = "imported_" .. tostring(getNextUniqueId())
      local isTimeBased = true
      if importedData.path[1].v ~= nil then
        isTimeBased = false
      end
      trajectories[tIdx] = createTrajectory(importedData.path, nil, nil, importedData.externalForce, isTimeBased)
      assignTrajWin(tIdx)
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Export a single trajectory to file (compatible with standard scriptAI scripts).
local function export(poly, tr)
  local nodes = buildNodes(poly, tr)
  local script = {}                                                             -- Populate the final script.
  script.path = nodes
  if tr.isExternalForce[0] == true then
    script.externalForce = true
  end
  extensions.editor_fileDialog.saveFile(                                        -- Write to file, with dialog.
    function(data)
      jsonWriteFile(data.filepath, script, true)
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Handles the mouse-ray node selection when the UI is in 'Draw In' mode.
local function handleMouseDrawIn()
  local ray = getCameraMouseRay()
  local dist = castRayStatic(ray.pos, ray.dir, 1000)
	local pos = ray.pos + ray.dir * dist
  local camPos = core_camera.getPosition()
  local dist = (pos - camPos):length()
  local dwd = drawWinData
  local color = ColorF(dwd.drawCol[0], dwd.drawCol[1], dwd.drawCol[2], 0.5)
  debugDrawer:drawSphere(pos, 0.1 * sqrt(dist), color)                          -- Draw a sphere at the current mouse position, on the map ground.
  local dn = dwd.drawNodes                                                      -- Draw the trajectory polyline, if there are at least two nodes.
  local numNodes = #dn
  if numNodes > 1 then
    for j = 2, numNodes do
      local n1, n2 = dn[j - 1], dn[j]
      local p1, p2 = vec3(n1.x, n1.y, n1.z), vec3(n2.x, n2.y, n2.z)
      debugDrawer:drawLineInstance(p1, p2, 4, color)
    end
    local nStart, nEnd = dn[1], dn[numNodes]                                    -- Draw start/end text, if there are at least two nodes.
    local pStart, pEnd = vec3(nStart.x, nStart.y, nStart.z), vec3(nEnd.x, nEnd.y, nEnd.z)
    debugDrawer:drawTextAdvanced(pStart, "Start", colors.textA, true, false, colors.textB)
    debugDrawer:drawTextAdvanced(pEnd, "End", colors.textA, true, false, colors.textB)
  end
  for j = 1, numNodes do                                                        -- Draw node spheres.
    local n = dn[j]
    local p = vec3(n.x, n.y, n.z)
    debugDrawer:drawSphere(p, 0.1 * sqrt(p:distance(camPos)), color)
  end
  if im.IsAnyItemHovered() == false and im.IsWindowHovered(im.HoveredFlags_AnyWindow) == false and im.IsMouseClicked(0) == true then
    local oldNodes = {}
    oldNodes = copy(dn)
    dn[numNodes + 1] = { x = pos.x, y = pos.y, z = pos.z, isLocked = false }    -- Handle left mouse clicks, to add nodes.
    local newNodes = {}
    newNodes = copy(dn)
    local data = { old = oldNodes, new = newNodes }
    editor.history:commitAction("Draw node", data, drawUndo, drawRedo)
  end
end

-- Computes the minimum distance between a given point and a given ray.
local function minDistBetweenPointAndRay(p, rayPos, rayDir)
  local pMinusB = p - rayPos
  local t0 = rayDir:dot(pMinusB) / rayDir:dot(rayDir)
  if t0 > 0 then
    return pMinusB:length()
  end
  return (p - (rayPos + (t0 * rayDir))):length()
end

-- Evaluates the camera path spline at a given time position.
local function calculateTnorm(d12, d23, d34, t1, t2, t3, t)
  return clamp((monotonicSteffen(0, d12, d12 + d23, d12 + d23 + d34, 0, t1, t1 + t2, t1 + t2 + t3, t1 + t) - d12) / d23, 0, 1)
end

-- Computes the position and rotation at a position on the camera path spline.
local function evalTCam(i2, tLoc, isComputeRot)
  local cwd = camWinData
  local nodes = cwd.nodes
  local posSmooth = nodes[i2].smoothness[0]
  local len = #nodes
  local i1, i3, i4 = max(i2 - 1, 1), min(i2 + 1, len), min(i2 + 2, len)
  local n1, n2, n3, n4 = nodes[i1], nodes[i2], nodes[i3], nodes[i4]             -- Get the four points surrounding the current time position on the working cam polyline.
  local dt1, dt2, dt3 = n2.t[0] - n1.t[0], n3.t[0] - n2.t[0], n4.t[0] - n3.t[0] -- The dt between each adjacent pair of nodes.
  local p1 = vec3(n1.x[0], n1.y[0], n1.z[0])
  local p2 = vec3(n2.x[0], n2.y[0], n2.z[0])
  local p3 = vec3(n3.x[0], n3.y[0], n3.z[0])
  local p4 = vec3(n4.x[0], n4.y[0], n4.z[0])
  if i1 == i2 then                                                              -- Edge case: Add a virtual marker at the start for p1, so the cam speed is smoother.
    local p23Len = (p3 - p2):length()
    if p23Len == 0 then
      p1 = p2
      dt1 = 0
    else
      local dir = catmullRomChordal(p1, p2, p3, p4, 0.1, posSmooth) - p2
      p1 = p2 - dir
      dt1 = dt2 * (dir:length() / p23Len)
    end
  end
  if i3 == i4 then                                                              -- Edge case: Add a virtual marker at the end for p4, so the cam speed is smoother.
  local p23Len = (p3 - p2):length()
    if p23Len == 0 then
      p4 = p3
      dt3 = 0
    else
      local dir = p3 - catmullRomChordal(p1, p2, p3, p4, 0.9, posSmooth)
      p4 = p3 + dir
      dt3 = dt2 * (dir:length() / p23Len)
    end
  end
  local tNorm = calculateTnorm(p1:distance(p2), p2:distance(p3), p3:distance(p4), dt1, dt2, dt3, tLoc)
  local pos = catmullRomChordal(p1, p2, p3, p4, tNorm, posSmooth)
  local rot = nil                                                               -- Get the rotations at each point, if requested.
  if isComputeRot == true then
    local r1 = quat(n1.qx, n1.qy, n1.qz, n1.qw)
    local r2 = quat(n2.qx, n2.qy, n2.qz, n2.qw)
    local r3 = quat(n3.qx, n3.qy, n3.qz, n3.qw)
    local r4 = quat(n4.qx, n4.qy, n4.qz, n4.qw)
    if i1 == i2 then                                                            -- Edge case: Set the correct rotation to the virtual marker at the start.
      local catmullRot = catmullRomCentripetal(r1, r2, r3, r4, 0.1)
      catmullRot:normalize()
      r1 = r2:nlerp(catmullRot, -1)
    end
    if i3 == i4 then                                                            -- Edge case: Set the correct rotation to the virtual marker at the end.
      local catmullRot = catmullRomCentripetal(r1, r2, r3, r4, 0.9)
      catmullRot:normalize()
      r4 = r3:nlerp(catmullRot, -1)
    end
    if r2:dot(r1) < 0 then
      r2 = -r2
    end
    if r3:dot(r2) < 0 then
      r3 = -r3
    end
    if r4:dot(r3) < 0 then
      r4 = -r4
    end
    local tNorm = calculateTnorm(sqrt(r1:distance(r2)), sqrt(r2:distance(r3)), sqrt(r3:distance(r4)), dt1, dt2, dt3, tLoc)
    rot = catmullRomCentripetal(r1, r2, r3, r4, tNorm)
    rot:normalize()
  end
  return pos, rot
end

-- Computes the velocities of each line segment in a trajectory.
local function computeCurrentVelocities(poly)
  local v = {}
  local len = #poly
  for i = 2, len do
    local iLast = i - 1
    local n0, n1 = poly[iLast], poly[i]
    v[iLast] = (vec3(n0.x[0], n0.y[0], n0.z[0]) - vec3(n1.x[0], n1.y[0], n1.z[0])):length() / (n1.t[0] - n0.t[0])
  end
  return v
end

-- Adjust the times of a trajectory to match those of a given velocity profile.
local function adjustTimesToMaintainVelocities(tIdx, nIdx, oldVel)
  local poly = getPolyRef(trajectories[tIdx])
  local nearestLockInPast = 1                                                   -- Compute nearest locked node in past (if none, then first node will be the locked node).
  for i = nIdx, 1, -1 do
    if poly[i].isLocked == true then
      nearestLockInPast = i
      break
    end
  end
  local t = poly[nearestLockInPast].t[0]                                        -- Adjust all times, from the first node after the nearest past locked node.
  local len = #poly
  for i = nearestLockInPast + 1, len do
    if poly[i].isLocked == true then
      break
    end
    local oldV = oldVel[i - 1]
    local n0, n1 = poly[i - 1], poly[i]
    local p0, p1 = vec3(n0.x[0], n0.y[0], n0.z[0]), vec3(n1.x[0], n1.y[0], n1.z[0])
    local d = (p0 - p1):length()
    t = t + d / max(1e-30, oldV)
    poly[i].t = im.FloatPtr(t)
  end
  local isRevert = false                                                        -- Check if the times are now out of order, or if times have blown up anywhere.
  for i = 2, len do
    local tNode = poly[i].t[0]
    if tNode < poly[i - 1].t[0] or abs(tNode) > 1e7 or tNode ~= poly[i].t[0] then
      isRevert = true
      break
    end
  end
  return isRevert
end

-- Compute the weights used for dragging a polyline with the force field, where an epicentre node is supplied.
local function getWeights(poly, fieldRange, epi)
  local FRInv, weights, len = 1.0 / fieldRange, {}, #poly
  weights[epi] = 1.0
  if epi < len then
    local total = 0.0
    for i = epi + 1, len do                                                     -- Move forward from the selected node, and compute the weights for every node there.
      local n1, n2 = poly[i - 1], poly[i]
      local d = (vec3(n2.x[0], n2.y[0], n2.z[0]) - vec3(n1.x[0], n1.y[0], n1.z[0])):length()
      total = total + d
      weights[i] = max(0, (fieldRange - total) * FRInv)
    end
  end
  if epi > 1 then
    local total = 0.0
    for i = epi - 1, 1, -1 do                                                   -- Move backwards from the selected node, and compute the weights for every node there.
      local n1, n2 = poly[i], poly[i + 1]
      local d = (vec3(n2.x[0], n2.y[0], n2.z[0]) - vec3(n1.x[0], n1.y[0], n1.z[0])):length()
      total = total + d
      weights[i] = max(0, (fieldRange - total) * FRInv)
    end
  end
  return weights
end

-- Draw a ghosted thick polyline around the current lock section of the vehicle trajectory.
local function highlightLockSectTraj(poly, l, u)
  if l == nil then                                                              -- If there is no lower bound, use the first node.
    l = 1
  end
  if u == nil then                                                              -- If there is no upper bound, use the last node.
    u = #poly
  end
  local len = u - 1
  for i = l, len do
    local n1, n2 = poly[i], poly[i + 1]
    debugDrawer:drawLineInstance(vec3(n1.x[0], n1.y[0], n1.z[0]), vec3(n2.x[0], n2.y[0], n2.z[0]), 12, colors.fField)
  end
end

-- Draws a ghosted thick polyline around the current lock section of the camera trajectory.
local function highlightLockSectCam(poly, l, u)
  if l == nil then                                                              -- If there is no lower bound, use the first node.
    l = 1
  end
  if u == nil then                                                              -- If there is no upper bound, use the last node.
    u = #poly
  end
  local n = poly[l]
  local pLast = vec3(n.x[0], n.y[0], n.z[0])
  local len = u - 1
  for j = l, len do
    local dt = poly[j + 1].t[0] - poly[j].t[0]
    for k = 0, 20 do
      local pNew, _ = evalTCam(j, (k * dt) * 0.05, false)
      debugDrawer:drawLineInstance(pLast, pNew, 12, colors.fField)
      pLast = pNew
    end
  end
end

-- Compute the interpolation positions for each node time in a lock section.
local function computeIntPosns(poly, l, u)
  local tL, tU = poly[l].t[0], poly[u].t[0]
  local dtInv, ip = 1.0 / (tU - tL), {}
  for i = l, u do
    ip[i] = (poly[i].t[0] - tL) * dtInv
  end
  return ip
end

-- Adjust node times based on their interpolation positions.
local function tAdjust(poly, d, l, u, off)
  local lLock, uLock, len = d.lLock, d.uLock, #poly
  if lLock == nil and uLock == nil then                                         -- [Case #1]: No locks on either side. Complete rigid translation.
    for i = 1, len do
      poly[i].t = im.FloatPtr(poly[i].t[0] + off)
    end
    return
  elseif lLock == nil then                                                      -- [Case #2]: Only upper lock.
    local ips = computeIntPosns(poly, l, uLock)                                 -- Before changes, note the interp positions of all nodes in [iLock, u].
    for i = 1, l do                                                             -- In [1, l], do a rigid time translation first.
      poly[i].t = im.FloatPtr(poly[i].t[0] + off)
    end
    local tL = poly[l].t[0]
    local dt = poly[uLock].t[0] - tL
    for i = l + 1, uLock - 1 do                                                 -- In [l, uLock - 1], do tension/compression based on interp positions.
      poly[i].t = im.FloatPtr(tL + ips[i] * dt)
    end
    return
  elseif uLock == nil then                                                      -- [Case #3]: Only lower lock.
    local ips = computeIntPosns(poly, lLock, u)                                 -- Before changes, note the interp positions of all nodes in [iLock, u].
    for i = u, len do                                                           -- In [u, end], do a rigid time translation first.
      poly[i].t = im.FloatPtr(poly[i].t[0] + off)
    end
    local tLLock = poly[lLock].t[0]
    local dt = poly[u].t[0] - tLLock
    for i = lLock + 1, u - 1 do                                                 -- In [lLock + 1, u], do tension/compression based on interp positions.
      poly[i].t = im.FloatPtr(tLLock + ips[i] * dt)
    end
    return
  end
  if uLock - lLock == 1 then                                                    -- Nothing can be done if we have consecutive locks.
    return
  end
  local tMid = d.tMid                                                           -- [Case #4]: There are locks on either side, so use tension/compression method.
  if off > 0.0 then                                                             -- Let the mouse control the size of the tension/compression interval.
    d.intV = min(d.intV + 0.01, 4.0)
  elseif off < 0.0 then
    d.intV = max(d.intV - 0.01, 0.1)
  end
  for k, v in pairs(d.ratios) do                                                -- Project each time position to the mouse-controlled interval.
    if k ~= lLock and k ~= uLock then
      poly[k].t = im.FloatPtr(tMid + (d.intV * v))
    end
  end
end

-- Handles the mouse-ray selection.
local function handleMouseSelection()
  -- Dragging is already in process, so do nothing here.
  if mState.isDragArmed == true then return end
  -- Get the position on the map where the mouse is raycast to.
  local ray = getCameraMouseRay()
	local d = castRayStatic(ray.pos, ray.dir, 1000)
	local pos = ray.pos + ray.dir * d
  -- First, check to see if the hit position is on any trajectory vehicle box.
  local isHovered = false
  for k, tr in pairs(trajectories) do
    local boxData = tr.boxData
    if boxData ~= nil then
      if boxData.pos ~= nil then
        local dist = (pos - boxData.pos):length()
        if dist < 3.0 then
          isHovered = true
          local poly = getPolyRef(tr)
          local lower, upper = getBounds(poly)
          local lLock, uLock = getLockBounds(poly, lower, upper)
          local ratios, tMid = {}, nil
          if lLock ~= nil and uLock ~= nil then
            tMid = (poly[lower].t[0] + poly[upper].t[0]) * 0.5
            local halfDtInv = 1.0 / (poly[upper].t[0] - tMid)
            for i = lLock, uLock do
              ratios[i] = (poly[i].t[0] - tMid) * halfDtInv
            end
          end
          highlightLockSectTraj(poly, lLock, uLock)
          drawVehBox(boxData, true)
          if im.IsAnyItemHovered() == false and im.IsWindowHovered(im.HoveredFlags_AnyWindow) == false and im.IsMouseClicked(0) == true then
            mState.vehSelectData = {
              boxData = boxData,
              lastPos = pos,
              trajectoryIdx = k,
              lLock = lLock, uLock = uLock,
              tMid = tMid, ratios = ratios, intV = 1.0 }
            mState.nodeSelectData, mState.camSelectData, mState.isDragArmed = nil, nil, true
          end
          return
        end
      end
    end
  end
  if isHovered == true then return end

  -- Second, check to see if the hit position is on the camera box.
  local cwd = camWinData
  local poly = cwd.nodes
  local len = #poly
  local isHovered = false
  if cwd.isDisplay[0] == true and len > 1  and toolWinData.isExecuting == false then
    local lower, upper = getBounds(poly)
    local tLoc = toolWinData.t[0] - poly[lower].t[0]
    local boxPos, boxRot = evalTCam(lower, tLoc, true)
    local dist = minDistPointLineSeg(boxPos, ray.pos, pos)
    if dist < 1.0 then
      isHovered = true
      local lLock, uLock = getLockBounds(poly, lower, upper)
      local ratios, tMid = {}, nil
      if lLock ~= nil and uLock ~= nil then
        tMid = (poly[lower].t[0] + poly[upper].t[0]) * 0.5
        local halfDtInv = 1.0 / (poly[upper].t[0] - tMid)
        for i = lLock, uLock do
          ratios[i] = (poly[i].t[0] - tMid) * halfDtInv
        end
      end
      highlightLockSectCam(poly, lLock, uLock)
      drawCamBox(boxPos, boxRot, true)
      if im.IsAnyItemHovered() == false and im.IsWindowHovered(im.HoveredFlags_AnyWindow) == false and im.IsMouseClicked(0) == true then
        mState.camSelectData = {
          lastPos = pos,
          lLock = lLock, uLock = uLock,
          tMid = tMid, ratios = ratios, intV = 1.0 }
        mState.nodeSelectData, mState.vehSelectData, mState.isDragArmed = nil, nil, true
      end
      return
    end
  end
  if isHovered == true then return end

  -- Find the closest node to the hit position, from all visible trajectories.
  local dClosest, trClosest, nClosest = 1e30, nil, nil
  for id, tr in pairs(trajectories) do
    if tr.isDisplay[0] == true then
      local sc = getPolyRef(tr)
      local len = #sc
      for j = 1, len do
        local n = sc[j]
        local p = vec3(n.x[0], n.y[0], n.z[0])
        local d = minDistBetweenPointAndRay(p, pos, ray.dir)
        if d < dClosest then
          dClosest, trClosest, nClosest = d, id, j
        end
      end
    end
  end
  -- If we have a valid closest node, then select/highlight it.
  if trClosest ~= nil and nClosest ~= nil then
    local tr = trajectories[trClosest]
    local isSpline = tr.isUseSpline[0]
    local poly = getPolyRef(tr)
    local n = poly[nClosest]
    local p = vec3(n.x[0], n.y[0], n.z[0])
    local hitToPointDist = (p - pos):length()
    if hitToPointDist < 5.0 then
      local camToPointDist = (p - core_camera.getPosition()):length()
      local nDist = 0.15 * sqrt(camToPointDist)
      if n.isLocked == true then
        local sqC = Point2F(nDist, nDist)
        debugDrawer:drawSquarePrism(p - vec3(0, 0, nDist), p + vec3(0, 0, nDist), sqC, sqC, colors.nGlow)
      else
        debugDrawer:drawSphere(p, nDist, colors.nGlow)
      end
      local lLock, uLock = getLockBounds(poly, nClosest, nClosest)
      highlightLockSectTraj(poly, lLock, uLock)
      if im.IsAnyItemHovered() == false and im.IsWindowHovered(im.HoveredFlags_AnyWindow) == false and im.IsMouseClicked(0) == true then
        trajectories[trClosest].selectedNode = nClosest
        local oldNodes, len = {}, #poly
        if tr.isTimeBased == true then
          for i = 1, len do
            local n = poly[i]
            oldNodes[i] = { x = n.x[0], y = n.y[0], z = n.z[0], t = n.t[0], isLocked = n.isLocked }
          end
        else
          for i = 1, len do
            local n = poly[i]
            oldNodes[i] = { x = n.x[0], y = n.y[0], z = n.z[0], v = n.v[0], isLocked = n.isLocked }
          end
        end
        local weights = getWeights(poly, tr.fieldRange[0], nClosest)
        mState.nodeSelectData = { trajectory = trClosest, node = nClosest, weights = weights, oldNodes = oldNodes, isSpline = isSpline }
        mState.vehSelectData, mState.camSelectData, mState.isDragArmed = nil, nil, true
      end
    end
  end
end

-- Perform time editing on a polyline.
local function handleTimeEdit(poly, d, l, u, mouseOffset)
  local len = #poly
  local tOld = {}
  for i = 1, len do                                                                 -- Cache the old time values, incase we need to revert.
    tOld[i] = poly[i].t[0]
  end
  tAdjust(poly, d, l, u, mouseOffset)
  local isRevert = false                                                            -- Test that avg velocities through all line segments are tolerable.
  for i = 2, len do
    local v = getVel(poly, i - 1, i)
    if v < 0.0 or v > 45.0 then
      isRevert = true
      break
    end
  end
  if isRevert == true then                                                          -- If limits were breached, revert all time values.
    for i = 1, len do
      poly[i].t = im.FloatPtr(tOld[i])
    end
  end
end

-- Handles the mouse dragging functionality.
local function handleMouseDragEditing()

  -- If the user is not currently dragging, there is nothing to do here.
  if mState.isDragArmed == false then return end

  -- Handle the mouse up event (only applies to trajectory nodes, to commit history).
  if im.IsMouseDown(0) == false then
    mState.isDragArmed = false
    if mState.nodeSelectData ~= nil then
      local newNodes, sc = {}, nil
      local tr = trajectories[mState.nodeSelectData.trajectory]
      if mState.nodeSelectData.isSpline == true then
        sc = tr.spline
      else
        sc = tr.polyLine
      end
      local len = #sc
      if tr.isTimeBased == true then
        for j = 1, len do
          local n = sc[j]
          newNodes[j] = { x = n.x[0], y = n.y[0], z = n.z[0], t = n.t[0], isLocked = n.isLocked }
        end
      else
        for j = 1, len do
          local n = sc[j]
          newNodes[j] = { x = n.x[0], y = n.y[0], z = n.z[0], v = n.v[0], isLocked = n.isLocked }
        end
      end
      local data = { old = mState.nodeSelectData.oldNodes, new = newNodes, isSpline = mState.nodeSelectData.isSpline, tIdx = mState.nodeSelectData.trajectory }
      editor.history:commitAction("Move Trajectory", data, nodesUndo, nodesRedo)
    end
    mState.nodeSelectData, mState.vehSelectData, mState.camSelectData = nil, nil, nil
    return
  end

  -- The mouse is still being dragged, so compute its latest position on the map.
  local ray = getCameraMouseRay()
  local pos = ray.pos + ray.dir * castRayStatic(ray.pos, ray.dir, 1000.0)

  -- Handle the case if a vehicle is selected.
  local vd = mState.vehSelectData
  if vd ~= nil then
    local tr = trajectories[vd.trajectoryIdx]
    local poly = getPolyRef(tr)
    local lLock, uLock = vd.lLock, vd.uLock
    highlightLockSectTraj(poly, lLock, uLock)
    drawVehBox(tr.boxData, true)
    local mouseY = pos.y - vd.lastPos.y                                               -- The difference in the Y-dimension of the mouse start position and current position.
    vd.lastPos = pos                                                                  -- Make the difference relative, so mouse stops acting if it does not move.
    local l, u = getBounds(poly)
    handleTimeEdit(poly, vd, l, u, mouseY)
    return
  end

  -- Handle the case if the camera is selected.
  local cd = mState.camSelectData
  if cd ~= nil then
    local cwd = camWinData
    local poly = cwd.nodes
    local lLock, uLock = cd.lLock, cd.uLock
    local l, u = getBounds(poly)
    local tLoc = toolWinData.t[0] - poly[l].t[0]
    local boxPos, boxRot = evalTCam(l, tLoc, true)
    highlightLockSectCam(poly, lLock, uLock)
    drawCamBox(boxPos, boxRot, true)
    local mouseY = pos.y - cd.lastPos.y                                               -- The difference in the Y-dimension of the mouse start position and current position.
    cd.lastPos = pos                                                                  -- Make the difference relative, so mouse stops acting if it does not move.
    handleTimeEdit(poly, cd, l, u, mouseY)
    return
  end

  -- Compute the current velocities for each line segment in the trajectory.
  local tIdx, nIdx, weights = mState.nodeSelectData.trajectory, mState.nodeSelectData.node, mState.nodeSelectData.weights
  local tr = trajectories[tIdx]
  -- Get the polyline (either the polyline or spline).
  local polyLine = getPolyRef(tr)
  -- Store the pre-change velocities, for later reference.
  local oldVelocities = nil
  if tr.isTimeBased == true then
    oldVelocities = computeCurrentVelocities(polyLine)
  end
  -- Determine whether to perform a fully-rigid translation upon the whole trajectory, or whether to move using the force field.
  local n = polyLine[nIdx]
  local len = #polyLine
  local pOld = vec3(n.x[0], n.y[0], n.z[0])
  local translation = pos - pOld
  if tr.isUseRigidTranslation[0] == true then
    for i = 1, len do                                                 -- Using rigid translation.
      local old = polyLine[i]
      polyLine[i].x, polyLine[i].y, polyLine[i].z = im.FloatPtr(old.x[0] + translation.x), im.FloatPtr(old.y[0] + translation.y), im.FloatPtr(old.z[0] + translation.z)
    end
  else
    -- Create a deep copy of the polyline before any changes are made, so we can revert to it if the times go out of order.
    local revPoly = {}
    if tr.isTimeBased == true then
      for i = 1, len do
        local n = polyLine[i]
        revPoly[i] = { x = n.x[0], y = n.y[0], z = n.z[0], t = n.t[0] }
      end
    else
      for i = 1, len do
        local n = polyLine[i]
        revPoly[i] = { x = n.x[0], y = n.y[0], z = n.z[0], v = n.v[0] }
      end
    end
    -- Draw the nodes and trajectory with a highlight.
    if n.isLocked == true then
      local camToPointDist = (pOld - core_camera.getPosition()):length()
      local nDist = 0.15 * sqrt(camToPointDist)
      local sqC = Point2F(nDist, nDist)
      debugDrawer:drawSquarePrism(pOld - vec3(0, 0, nDist), pOld + vec3(0, 0, nDist), sqC, sqC, colors.nGlow)
    else
      local camToPointDist = (pos - core_camera.getPosition()):length()
      local nDist = 0.15 * sqrt(camToPointDist)
      debugDrawer:drawSphere(pOld, nDist, colors.nGlow)
    end
    local lLock, uLock = getLockBounds(polyLine, nIdx, nIdx)
    highlightLockSectTraj(polyLine, lLock, uLock)
    -- Handle the moving of the nodes.
    for i = nIdx, 1, -1 do                                                  -- Propagate backwards from selected node: using force field weighting for time translation.
      local old = polyLine[i]
      if old.isLocked == true then                                          -- Only propagate until reaching a locked node, then leave.
        break
      end
      local tra = translation * weights[i]
      polyLine[i].x, polyLine[i].y, polyLine[i].z = im.FloatPtr(old.x[0] + tra.x), im.FloatPtr(old.y[0] + tra.y), im.FloatPtr(old.z[0] + tra.z)
    end
    if nIdx < len then
      for i = nIdx + 1, len do                                              -- Propagate forwards from selected node: using force field weighting for time translations.
        local old = polyLine[i]
        if old.isLocked == true then                                        -- Only propagate until reaching a locked node, then leave.
          break
        end
        if i == nIdx + 1 and polyLine[nIdx].isLocked == true then           -- Special Case:  We start from next node, so we must check the selected node's locking value.
          break
        end
        local tra = translation * weights[i]
        polyLine[i].x, polyLine[i].y, polyLine[i].z = im.FloatPtr(old.x[0] + tra.x), im.FloatPtr(old.y[0] + tra.y), im.FloatPtr(old.z[0] + tra.z)
      end
    end
    if tr.isTimeBased == true and tr.isHoldVelocity[0] == true then
      -- Adjust the node times to better match the new positioning (try to maintain old velocity).  If the polyline is out of order or the times have blown up, then revert.
      local isRevert = adjustTimesToMaintainVelocities(tIdx, nIdx, oldVelocities)
      if isRevert == true then
        for i = 1, len do
          polyLine[i].x, polyLine[i].x, polyLine[i].z, polyLine[i].t = im.FloatPtr(revPoly[i].x), im.FloatPtr(revPoly[i].y), im.FloatPtr(revPoly[i].z), im.FloatPtr(revPoly[i].t)
        end
      end
    end
  end
end

-- Handles the axis gizmo for translation and rotation of camera path nodes.
local function handleCamGizmo()
  local cwd = camWinData
  if cwd.isDisplay[0] == false or #cwd.nodes < 1 or toolWinData.isExecuting == true or cwd.selectedNode == nil or cwd.selectedNode < 1 then return end
  local n = cwd.nodes[cwd.selectedNode]
  local rotation = QuatF(0, 0, 0, 1)
  local transform = rotation:getMatrix()
  transform:setPosition(vec3(n.x[0], n.y[0], n.z[0]))
  editor.setAxisGizmoTransform(transform)
  editor.updateAxisGizmo(gizmoBeginDrag, gizmoEndDrag, gizmoDragging)
  editor.drawAxisGizmo()
end

-- Draws the camera trajectory polyline.
local function drawCamTrajPolyline(pn, color)
  local pLast = vec3(pn[1].x[0], pn[1].y[0], pn[1].z[0])
  local len = #pn - 1
  for j = 1, len do
    local dt = pn[j + 1].t[0] - pn[j].t[0]
    for k = 0, 20 do
      local pNew, _ = evalTCam(j, (k * dt) * 0.05, false)
      debugDrawer:drawLineInstance(pLast, pNew, 4, color)
      pLast = pNew
    end
  end
end

-- Draw all the trajectories (which are set to display) on the main screen.
local function drawTrajectories()
  -- Iterate over the full array of trajectories and ignore those which are not marked for display.
  for k, tr in pairs(trajectories) do
    local c = tr.col
    local color = ColorF(c[0], c[1], c[2], 0.5)
    if tr.isDisplay[0] == true then
      local poly = getPolyRef(tr)
      local numNodes = #poly
      if numNodes > 1 then
        -- Draw the trajectory polyline.
        for j = 2, numNodes do
          local n1, n2 = poly[j - 1], poly[j]
          local p1 = vec3(n1.x[0], n1.y[0], n1.z[0])
          local p2 = vec3(n2.x[0], n2.y[0], n2.z[0])
          debugDrawer:drawLineInstance(p1, p2, 4, color)
        end
        -- Draw node spheres.
        local camPos = core_camera.getPosition()
        for j = 1, numNodes do
          local n = poly[j]
          local p = vec3(n.x[0], n.y[0], n.z[0])
          local dist = (p - camPos):length()
          local nDist = sqrt(dist) * 0.1
          if n.isLocked == true then
            local sqC = Point2F(nDist, nDist)
            debugDrawer:drawSquarePrism(p - vec3(0, 0, nDist), p + vec3(0, 0, nDist), sqC, sqC, color)
          else
            debugDrawer:drawSphere(p, nDist, color)
          end
        end
        -- Draw start/end text.
        local nStart, nEnd = poly[1], poly[numNodes]
        local pStart, pEnd = vec3(nStart.x[0], nStart.y[0], nStart.z[0]), vec3(nEnd.x[0], nEnd.y[0], nEnd.z[0])
        debugDrawer:drawTextAdvanced(pStart, "Start", colors.textA, true, false, colors.textB)
        debugDrawer:drawTextAdvanced(pEnd, "End", colors.textA, true, false, colors.textB)
        -- Mark all the trajectory nodes, if selected to do so.
        if tr.isMarkNodes[0] == true then
          for j = 1, numNodes do
            local n = poly[j]
            local p = vec3(n.x[0], n.y[0], n.z[0])
            debugDrawer:drawTextAdvanced(p, tostring(j), colors.textA, true, false, colors.textB)
          end
        end
        -- Mark all the trajectory velocities, if selected to do so.
          if tr.isMarkVelocities[0] == true then
            if numNodes > 1 then
              if tr.isTimeBased == true then
                for j = 2, numNodes do
                  local n1, n2 = poly[j - 1], poly[j]
                  local p1, p2 = vec3(n1.x[0], n1.y[0], n1.z[0]), vec3(n2.x[0], n2.y[0], n2.z[0])
                  local lineSegVec = p2 - p1
                  local d, mid = lineSegVec:length(), p1 + lineSegVec * 0.5
                  local velKph = (d * 3.6) / (n2.t[0] - n1.t[0])
                  debugDrawer:drawTextAdvanced(mid, tostring(round1(velKph)) .. ' kph', colors.textA, true, false, colors.textB)
                end
              else
                for j = 1, numNodes do
                  local n = poly[j]
                  local p = vec3(n.x[0], n.y[0], n.z[0])
                  debugDrawer:drawTextAdvanced(p, tostring(round1(n.v[0] * 3.6)) .. ' kph', colors.textA, true, false, colors.textB)
                end
              end
            end
          end
        -- If time-based, then draw the vehicle bounding box.
        if tr.isTimeBased == true then
          -- If attached, get the vehicle dimensions, otherwise use some default dimensions.
          local length, width, height = 5.0, 3.0, 2.0
          if tr.vid ~= nil then
            local obj = scenetree.findObject(tr.vid)
            if obj ~= nil then
              length, width, height = obj:getInitialLength(), obj:getInitialWidth(), obj:getInitialHeight()
            end
          end
          -- Draw the bounding-boxes in-place (at the current time) on each trajectory.
          local p1, p2, a = lerpTraj(tr)
          local pos = p1 + (p2 - p1) * a
          local dir = (p2 - p1)
          dir:normalize()
          tr.boxData = { pos = pos, dir = dir, width = width, length = length, height = height }
          drawVehBox(tr.boxData, false)
        end
      end
    end
  end
  -- Display the camera trajectory if it has been selected for display
  local cwd = camWinData
  local numNodes = #cwd.nodes
  if toolWinData.isExecuting == false and cwd.isDisplay[0] == true and numNodes > 0 then
    local c = cwd.col
    local color = ColorF(c[0], c[1], c[2], 0.5)
    -- Draw the camera trajectory polyline.
    local pn = cwd.nodes
    drawCamTrajPolyline(pn, color)
    -- Draw node spheres and text.
    local camPos = core_camera.getPosition()
    for j = 1, numNodes do
      local n = pn[j]
      local p = vec3(n.x[0], n.y[0], n.z[0])
      local dist = (p - camPos):length()
      local nDist = sqrt(dist) * 0.1
      if n.isLocked == true then
        local sqC = Point2F(nDist, nDist)
        debugDrawer:drawSquarePrism(p - vec3(0, 0, nDist), p + vec3(0, 0, nDist), sqC, sqC, color)
      else
        debugDrawer:drawSphere(p, nDist, color)
      end
      debugDrawer:drawTextAdvanced(p, tostring(j), colors.textA, true, false, colors.textB)
    end
    local pStart, pEnd = vec3(pn[1].x[0], pn[1].y[0], pn[1].z[0]), vec3(pn[numNodes].x[0], pn[numNodes].y[0], pn[numNodes].z[0])
    debugDrawer:drawTextAdvanced(pStart, "Start", colors.textA, true, false, colors.textB)
    debugDrawer:drawTextAdvanced(pEnd, "End", colors.textA, true, false, colors.textB)
    local lower, _ = getBounds(cwd.nodes)
    local tLoc = toolWinData.t[0] - cwd.nodes[lower].t[0]
    local pos, rot = evalTCam(lower, tLoc, true)
    drawCamBox(pos, rot, false)
  end
end

-- Converts a polyline/spline to an array of vec3 type.
local function traj2Vec3(d)
  local p, numNodes = {}, #d
  for i = 1, numNodes do
    local n = d[i]
    p[i] = vec3(n.x[0], n.y[0], n.z[0])
  end
  return p
end

-- Computes the tangents used in the fitting of parametric cubic polynomials.
local function computeTangents(pn0, pn1, pn2, pn3)
  local d1, d2, d3 = max(sqrt(pn0:distance(pn1)), 1e-12), sqrt(pn1:distance(pn2)), max(sqrt(pn2:distance(pn3)), 1e-12)
  local m = (pn1 - pn0) / d1 + (pn0 - pn2) / (d1 + d2)
  local n = (pn1 - pn3) / (d2 + d3) + (pn3 - pn2) / d3
  local pn12 = pn2 - pn1
  local t1 = (d2 * m) + pn12
  if t1:length() < 1e-5 then t1 = pn12 * 0.5 end
  local t2 = (d2 * n) + pn12
  if t2:length() < 1e-5 then t2 = pn12 * 0.5 end
  return t1, t2
end

-- Fits a series of parametric cubic polynomials to an array of coordinates.
local function fitParaCubic(coords)
  local cubics, ctr, numCoords = {}, 1, #coords
  for i1 = 1, numCoords - 1 do
    local i0, i2, i3 = max(i1 - 1, 1), i1 + 1, min(i1 + 2, numCoords)
    local seg0, seg1, seg2, seg3 = i0, i1, i2, i3
    local p1_2d = coords[seg1]
    local pn0_2d, pn1_2d, pn2_2d, pn3_2d = coords[seg0] - p1_2d, vec3(0.0, 0.0, 0.0), coords[seg2] - p1_2d, coords[seg3] - p1_2d
    local t1, t2 = computeTangents(pn0_2d, pn1_2d, pn2_2d, pn3_2d)
    local coeffC, coeffD = (-2.0 * t1) - t2 + (3.0 * pn2_2d), t1 + t2 - (2.0 * pn2_2d)
    cubics[ctr] = {
      uA = pn1_2d.x, uB = t1.x, uC = coeffC.x, uD = coeffD.x,
      vA = pn1_2d.y, vB = t1.y, vC = coeffC.y, vD = coeffD.y,
      wA = pn1_2d.z, wB = t1.z, wC = coeffC.z, wD = coeffD.z  }
    ctr = ctr + 1
  end
  return cubics
end

-- Removes duplicate nodes from a fitted spline.
local function removeDuplicatesTraj(d)
  local len, proc, ctr = #d, {}, 2
  proc[1] = d[1]
  for i = 2, len do
    local n1, n2 = d[i - 1], d[i]
    local p1, p2 = vec3(n1.x, n1.y, n1.z), vec3(n2.x, n2.y, n2.z)
    if (p2 - p1):squaredLength() > 1e-7 then
      proc[ctr] = n2
      ctr = ctr + 1
    end
  end
  return proc
end

-- Fit a spline to the trajectory with the given index, and re-compute a smoother polyline.
local function fitSplineToTraj(tr)
  local poly = tr.polyLine
  local coords = traj2Vec3(poly)
  local cubics = fitParaCubic(coords)
  local nodes, ctr, numCubics = {}, 1, #cubics
  local dx = tr.splineSpacing[0]
  local dxInv = 1.0 / dx
  for i = 1, numCubics do
    local c = cubics[i]
    local p1 = coords[i]
    local xStart, yStart, zStart = p1.x, p1.y, p1.z
    for j = 0, dx do
      local t = j * dxInv
      local t2 = t * t
      local t3 = t2 * t
      local u = c.uA + (t * c.uB) + (t2 * c.uC) + (t3 * c.uD)
      local v = c.vA + (t * c.vB) + (t2 * c.vC) + (t3 * c.vD)
      local w = c.wA + (t * c.wB) + (t2 * c.wC) + (t3 * c.wD)
      local v1 = vec3(xStart + u, yStart + v, zStart + w)
      nodes[ctr] = { x = v1.x, y = v1.y, z = v1.z, isLocked = false }
      ctr = ctr + 1
    end
  end
  local nProc = removeDuplicatesTraj(nodes)
  local procNodes = nil
  if tr.isTimeBased == true then
    procNodes = setFixedVelValT(nProc, 8.33333333)
  else
    procNodes = {}
    local len, polyLen, spacInv = #nodes, #poly, 1.0 / tr.splineSpacing[0]
    for i = 1, len do
      procNodes[i] = nodes[i]
      local sampleIdx = min(polyLen, max(1, floor(i * spacInv)))
      procNodes[i].v = im.FloatPtr(poly[sampleIdx].v[0])
      procNodes[i] = val2PtrV(procNodes[i])
    end
  end
  -- Set all the nodes to be unlocked.
  for i = 1, #procNodes do
    procNodes[i].isLocked = false
  end
  -- Set the dir and up vectors. If they are not provided, compute them from the trajectory.
  local dirVec = (vec3(procNodes[2].x[0], procNodes[2].y[0], procNodes[2].z[0]) - vec3(procNodes[1].x[0], procNodes[1].y[0], procNodes[1].z[0]))
  dirVec:normalize()
  procNodes[1].dir = { x = dirVec.x, y = dirVec.y, z = dirVec.z }
  procNodes[1].up = { x = 0.0, y = 0.0, z = 1.0 }
  -- Set the spline data.
  tr.spline = procNodes
end

local function onEditorGui()
  if not editor.editMode or editor.editMode.displayName ~= editModeName then return end

  -- Compute the scenario interval.
  local numTrajectories = 0
  for k, tr in pairs(trajectories) do
    numTrajectories = numTrajectories + 1
  end
  if trajectories == nil or numTrajectories < 1 then
    toolWinData.tStart, toolWinData.tEnd = 0.0, 0.0                               -- No trajectories loaded, so use [0, 0].
  else
    toolWinData.tStart, toolWinData.tEnd = 0.0, 0.0
    for k, tr in pairs(trajectories) do
      local polyLine = getPolyRef(tr)
      local len = #polyLine
      if tr.vehicle ~= "" and polyLine ~= nil and len > 0 then                    -- Only include trajectories which are attached to vehicles.
        if tr.isTimeBased == true then
          toolWinData.tStart = min(toolWinData.tStart, polyLine[1].t[0])
          toolWinData.tEnd = max(toolWinData.tEnd, polyLine[len].t[0])
        else
          toolWinData.tStart = min(toolWinData.tStart, tr.vModeTStart[0])
          toolWinData.tEnd = max(toolWinData.tEnd, tr.vModeTEnd[0])
        end
      end
    end
  end
  -- Manage the time transport evolution.
  local dt = timer:stopAndReset() * 0.001
  if toolWinData.isPlaying == true then
    toolWinData.t = im.FloatPtr(toolWinData.t[0] + dt)
    if toolWinData.t[0] >= toolWinData.tEnd then
      local cwd = camWinData
      if cwd.isOnExecute[0] == true and #cwd.nodes > 1 then
        core_paths.stopCurrentPath()
      end
      if toolWinData.isLooping[0] == true then
        toolWinData.t = im.FloatPtr(toolWinData.tStart)
        if toolWinData.isExecuting == true then
          execute()
        end
      else
        toolWinData.t = im.FloatPtr(toolWinData.tEnd)
        toolWinData.isPlaying = false
        toolWinData.isExecuting = false
      end
    end
  end
  if toolWinData.t[0] > toolWinData.tEnd then
    local cwd = camWinData
    if cwd.isOnExecute[0] == true and #cwd.nodes > 1 then
      core_paths.stopCurrentPath()
    end
    if toolWinData.isLooping[0] == true then
      toolWinData.t = im.FloatPtr(toolWinData.tStart)
      if toolWinData.isExecuting == true then
        execute()
      end
    else
      toolWinData.t = im.FloatPtr(toolWinData.tEnd)
      toolWinData.isExecuting = false
    end
  end
  -- Create the vehicles list.
  sceneVehicles = {}
  local ctr = 1
  for vid, veh in activeVehiclesIterator() do
    local vehName, jBeam, config = veh:getName(), veh.JBeam, veh:getField('partConfig', '0')
    sceneVehicles[ctr] = {
      vid = vid,
      veh = veh,
      name = vehName,
      jBeam = jBeam,
      config = config,
      string = tostring(vid .. ": " .. vehName .. " - " .. jBeam) }
    if vehWinData.isRecording[ctr] == nil then
      vehWinData.isRecording[ctr] = false
    end
    ctr = ctr + 1
  end
  local numVehicles = #sceneVehicles

  -- Display the Main Tool Window.
  local twd = toolWinData
  if editor.beginWindow(twd.name, "Script AI Editor") then
    -- Draw the time transport slider.
    im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
    im.PushItemWidth(549)
    im.SliderFloat("", twd.t, twd.tStart, twd.tEnd, "time = %.3f / [" .. round2(twd.tStart) .. ", " .. round2(twd.tEnd) .."] s")
    im.PopItemWidth()
    im.PopStyleVar()

    im.Columns(2, "toolWindowCols", false)
    im.SetColumnWidth(0, 45)
    im.SetColumnWidth(1, 544)
    if twd.tStart < 0.0 then
      if editor.uiIconImageButton(editor.icons.watch_later, im.ImVec2(28, 28), nil, nil, nil, 'normalizeTrajectory') then
        for k, tr in pairs(trajectories) do
          local poly = getPolyRef(tr)
          local len = #poly
          for i = 1, len do
            poly[i].t = im.FloatPtr(poly[i].t[0] - twd.tStart)
          end
        end
        local nLen = #camWinData.nodes
        for i = 1, nLen do
          camWinData.nodes[i].t = im.FloatPtr(camWinData.nodes[i].t[0] - twd.tStart)
        end
      end
      im.tooltip('Scenario contains negative times. Please normalize before execution.')
      im.SameLine()
    elseif twd.tStart >= 0.0 and twd.tEnd > 0.0 then
      if twd.isExecuting == false then
        if editor.uiIconImageButton(editor.icons.movieCamera, im.ImVec2(30, 30), nil, nil, nil, 'execute') then
          twd.t = im.FloatPtr(twd.tStart)
          execute()
          twd.isPlaying = true
          twd.isExecuting = true
        end
        im.tooltip('Execute scenario')
      else
        if editor.uiIconImageButton(editor.icons.videocam_off, im.ImVec2(30, 30), nil, nil, nil, 'stopExecutution') then
          stopExecute()
          twd.isPlaying = false
          twd.isExecuting = false
        end
        im.tooltip('Stop scenario execution')
      end
      im.SameLine()
    end

    im.Dummy(im.ImVec2(5, 0))

    im.NextColumn()

    im.Checkbox("Overlay", twd.isOverlay)
    im.tooltip('Execute other scripts during recording')

    im.SameLine()
    im.Checkbox("Display", twd.isDispInExe)
    im.tooltip('Show trajectory guides when executing')

    im.SameLine()
    im.Dummy(im.ImVec2(5, 0))

    im.SameLine()
    if im.BeginListBox("", im.ImVec2(103, 31), im.WindowFlags_ChildWindow) then
      if twd.isExecuting == false then
        if twd.isPlaying == false then
          if editor.uiIconImageButton(editor.icons.play_arrow, im.ImVec2(28, 28), colors.black, nil, nil, 'play') then
            if abs(twd.t[0] - twd.tEnd) < 1e-3 then twd.t = im.FloatPtr(twd.tStart) end
            twd.isPlaying = true
          end
          im.tooltip('Play time-transport guide')
        else
          if editor.uiIconImageButton(editor.icons.stop, im.ImVec2(28, 28), colors.black, nil, nil, 'stop') then
            twd.isPlaying = false
          end
          im.tooltip('Stop time-transport guide')
        end

        im.SameLine()
        im.PushButtonRepeat(true)
        im.PushStyleVar2(im.StyleVar_CellPadding, im.ImVec2(0,-10))
        if editor.uiIconImageButton(editor.icons.fast_rewind, im.ImVec2(28, 28), colors.black, nil, nil, 'rewind') then
          twd.t = im.FloatPtr(twd.t[0] - twd.rewJump)
          if twd.t[0] <= 0.0 then
            twd.t = im.FloatPtr(0.0)
            twd.isPlaying = false
          end
        end
        im.tooltip('Rewind time-transport guide')
        im.PopStyleVar()

        im.SameLine()
        im.PushButtonRepeat(true)
        if editor.uiIconImageButton(editor.icons.fast_forward, im.ImVec2(28, 28), colors.black, nil, nil, 'fastForward') then
          twd.t = im.FloatPtr(twd.t[0] + twd.ffwdJump)
          if twd.t[0] >= twd.tEnd then
            twd.t = im.FloatPtr(twd.tEnd)
            twd.isPlaying = false
          end
        end
        im.tooltip('Fast forward time-transport guide')
      end
      im.EndListBox()
    end

    im.SameLine()
    im.Checkbox("Loop", twd.isLooping)
    im.tooltip('Loop the time-transport guide')

    im.SameLine()
    im.Dummy(im.ImVec2(5, 0))

    im.SameLine()
    if editor.uiIconImageButton(editor.icons.car, im.ImVec2(29, 29), im.ImVec4(1, 1, 1, winStates.vehWin.buttonAlpha), nil, nil, 'openVehiclesWindow') then
      if editor.isWindowVisible(vehWinData.name) then
        editor.hideWindow(vehWinData.name)
        winStates.vehWin.buttonAlpha = 1
      else
        editor.showWindow(vehWinData.name)
        winStates.vehWin.buttonAlpha = 0.5
      end
    end
    im.tooltip('Open/close vehicles window')

    im.SameLine()
    if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, im.ImVec2(27, 27), im.ImVec4(1, 1, 1, winStates.trajWin.buttonAlpha), nil, nil, 'openTrajectoriesWindow') then
      if editor.isWindowVisible(trajWinData.name) then
        editor.hideWindow(trajWinData.name)
        winStates.trajWin.buttonAlpha = 1.0
      else
        editor.showWindow(trajWinData.name)
        winStates.trajWin.buttonAlpha = 0.5
      end
    end
    im.tooltip('Open/close trajectories window')

    im.SameLine()
    if editor.uiIconImageButton(editor.icons.switch_video, im.ImVec2(29, 29), im.ImVec4(1, 1, 1, winStates.camWin.buttonAlpha), nil, nil, 'openCameraWindow') then
      if editor.isWindowVisible(camWinData.name) then
        editor.hideWindow(camWinData.name)
        winStates.camWin.buttonAlpha = 1
      else
        editor.showWindow(camWinData.name)
        winStates.camWin.buttonAlpha = 0.5
      end
    end
    im.tooltip('Open/close camera window')

    im.SameLine()
    im.Dummy(im.ImVec2(5, 0))

    im.SameLine()
    if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(26, 26), nil, nil, nil, 'loadSession') then
      load()
    end
    im.tooltip('Load session')
    if numTrajectories > 0 and numVehicles > 0 then
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.floppyDisk, im.ImVec2(24, 24), nil, nil, nil, 'saveSession') then
        save()
      end
      im.tooltip('Save session')
    end
    im.NextColumn()
  end
  editor.endWindow()

  -- Manage the vehicles window.
  if editor.isWindowVisible(vehWinData.name) and numVehicles > 0 and drawWinData.isDrawIn == false and twd.isExecuting == false then
    -- Compute a string for each vehicle, which represents its attachment to a trajectory, or no attachment.
    --TODO: maybe cache this and not do it every frame ?
    local vehicleAttachStates = {}
    for i = 1, numVehicles do
      local vehicle = sceneVehicles[i]
      local vehicleAttachState = "[not attached]"
      for k, tr in pairs(trajectories) do
        if tr.vehicle ~= nil and tr.vehicle == vehicle.string then
          vehicleAttachState = "Attached Trajectory: [" .. tostring(k) .. "]"
          break
        end
      end
      vehicleAttachStates[i] = vehicleAttachState
    end
    -- Display the vehicles window.
    local vwd = vehWinData
    if editor.beginWindow(vwd.name, "Scene Vehicles") then
      im.Separator()
      if im.BeginListBox("", im.ImVec2(480, 180), im.WindowFlags_ChildWindow) then
        local ctr = 1
        for i = numVehicles, 1, -1 do
          local vehicle = sceneVehicles[i]
          im.Columns(5, "columns3", false)
          im.SetColumnWidth(0, 180)
          im.SetColumnWidth(1, 30)
          im.SetColumnWidth(2, 30)
          im.SetColumnWidth(3, 30)
          im.SetColumnWidth(4, 200)
          local flag = false
          if i == vwd.selectedVeh then
            flag = true
          end
          if im.Selectable1(vehicle.string, flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            vwd.selectedVeh = i
          end
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.trashBin2, im.ImVec2(22, 22), nil, nil, nil, 'removeVehicle') then
            local selectedVehicle = sceneVehicles[i]
            selectedVehicle.veh:delete()
            table.remove(sceneVehicles, i)
            for tIdx, tr in pairs(trajectories) do
              if tr.vid == selectedVehicle.vid then
                tr.vehicle, tr.vid, tr.jBeam = "", nil, nil
              end
            end
            vwd.selectedVeh = 1
            return
          end
          im.tooltip('Remove this vehicle.')
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.car, im.ImVec2(21, 21), nil, nil, nil, 'goToSelectedVehicle') then
            core_camera.setByName(0, "orbit", false)
            be:enterVehicle(0, scenetree.findObject(vehicle.vid))
          end
          im.tooltip('Go to the selected vehicle.')
          im.NextColumn()
          if vehicleAttachStates[i] ~= "[not attached]" then
            local idx = nil
            for k, tr in pairs(trajectories) do
              if vehicle.vid == tr.vid then
                idx = k
              end
            end
            local btnCol = getTrajButtonCol(idx)
            if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, im.ImVec2(19, 19), btnCol, nil, nil, 'editTrajectoryVehicleWindow') then
              if idx ~= nil then
                assignTrajWin(idx)
              end
            end
            im.tooltip('Open the linked trajectory window.')
          end
          im.NextColumn()
          local vas = vehicleAttachStates[i]
          if vas == '[not attached]' then
            local veh = sceneVehicles[i]
            if vwd.isRecording[i] == false then
              if editor.uiIconImageButton(editor.icons.fiber_manual_record, im.ImVec2(21, 21), colors.rec, nil, nil, 'startRecordingScript') then
                core_camera.setByName(0, "orbit", false)
                be:enterVehicle(0, scenetree.findObject(veh.vid))
                if twd.isOverlay[0] == true then
                  execute()
                end
                if vwd.recordMode[i][0] == 2 then
                  scenetree.findObject(veh.vid):queueLuaCommand('ai.startRecording(true)')    -- Record a speed-based script.
                else
                  scenetree.findObject(veh.vid):queueLuaCommand('ai.startRecording()')        -- Record a time-based script.
                end
                twd.t = im.FloatPtr(twd.tStart)
                twd.isPlaying, vwd.isRecording[i] = true, true
              end
              im.tooltip('Record a script with this vehicle.')
              im.SameLine()
              im.Dummy(im.ImVec2(5, 0))
              im.SameLine()
              if vwd.recordMode[i] == nil then vwd.recordMode[i] = im.IntPtr(1) end
              if im.RadioButton2("T-Based###" .. tostring(ctr), vwd.recordMode[i], im.Int(1)) then end
              im.SameLine()
              if im.RadioButton2("V-Based###" .. tostring(ctr + 1), vwd.recordMode[i], im.Int(2)) then end
              ctr = ctr + 2
            else
              if editor.uiIconImageButton(editor.icons.stop, im.ImVec2(21, 21), nil, nil, nil, 'stopRecordingScript') then
                scenetree.findObject(veh.vid):queueLuaCommand('obj:queueGameEngineLua("extensions.hook(\\"onVehicleSubmitRecording\\","..tostring(objectId)..","..serialize(ai.stopRecording())..")")')
                stopExecute()
                twd.isPlaying, vwd.isRecording[i] = false, false
              end
              im.tooltip('Stop recording.')
            end
          end
          im.NextColumn()
          im.Separator()
        end
        im.EndListBox()
      end
      im.Separator()
      -- Display the attachment state of the currently-selected vehicle at the bottom of the window.
      im.Text(vehicleAttachStates[vwd.selectedVeh])
    else
      winStates.vehWin.buttonAlpha = 1
    end
    editor.endWindow()
  end

  -- Manage the trajectory list window.
  local tlwd = trajWinData
  if editor.isWindowVisible(tlwd.name) and numVehicles > 0 and drawWinData.isDrawIn == false and twd.isExecuting == false then
    if editor.beginWindow(tlwd.name, "Scene Trajectories") then
      im.Separator()
      if im.BeginListBox("", im.ImVec2(170, 180), im.WindowFlags_ChildWindow) then
        for k, tr in pairs(trajectories) do
          im.Columns(3, "columns3", false)
          im.SetColumnWidth(0, 80)
          im.SetColumnWidth(1, 30)
          im.SetColumnWidth(2, 30)
          local flag = false
          if k == tlwd.selectedTraj then flag = true end
          if im.Selectable1(tostring(k), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            tlwd.selectedTraj = k
          end
          im.NextColumn()
          if editor.uiIconImageButton(editor.icons.trashBin2, im.ImVec2(22, 22), nil, nil, nil, 'removeTrajectory') then
            local oldTrajData = serializeTrajData()
            trajectories[k] = nil
            tlwd.selectedTraj = nil
            if indTrajWinData.idx1 == k then
              indTrajWinData.idx1 = nil
            end
            if indTrajWinData.idx2 == k then
              indTrajWinData.idx2 = nil
            end
            if indTrajWinData.idx3 == k then
              indTrajWinData.idx3 = nil
            end
            local newTrajData = serializeTrajData()
            local data = { old = oldTrajData, new = newTrajData }
            editor.history:commitAction("Remove trajectory.", data, trajWinUndo, trajWinRedo)
            return
          end
          im.tooltip('Remove the selected trajectory.')
          im.NextColumn()
          if numTrajectories > 0 and tr ~= nil then
            local btnCol = getTrajButtonCol(k)
            if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, im.ImVec2(19, 19), btnCol, nil, nil, 'editTrajectoryTrajectoryWindow') then
              assignTrajWin(k)
            end
            im.tooltip('Open the linked trajectory window.')
          end
          im.NextColumn()
          im.Separator()
        end
        im.EndListBox()
      end
      im.Separator()
      -- Get a string which indicates if there is a vehicle with this trajectory attached to it, or not.
      local selectedTraj = trajectories[tlwd.selectedTraj]
      local trajectoryState = nil
      if selectedTraj == nil or selectedTraj.vehicle == nil or selectedTraj.vehicle == "" then
        trajectoryState = "Attach Status:\n[not attached]"
      else
        trajectoryState = "Attach Status:\n[" .. selectedTraj.vehicle .. "]"
      end
      im.Text(trajectoryState)
      im.Separator()
      if editor.uiIconImageButton(editor.icons.mode_edit, im.ImVec2(23, 23), nil, nil, nil, 'drawInTrajectoryManually') then
        editor.showWindow(drawWinData.name)
        drawWinData.drawCol = getNextColor()
        drawWinData.isDrawIn = true
      end
      im.tooltip('Manually draw-in a new trajectory on the map.')
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(23, 23), nil, nil, nil, 'importTrajectory') then
        import()
      end
      im.tooltip('Import a trajectory from file.')
    else
      winStates.trajWin.buttonAlpha = 1
    end
    editor.endWindow()
  end

  -- Display the three Individual Trajectory Windows.
  local itwd = indTrajWinData
  if numVehicles > 0 and drawWinData.isDrawIn == false and twd.isExecuting == false then
    for i = 1, 3 do
      -- Get the trajectory associated to the window indexed by the outer iterator.
      local tIdx, windowName = nil, nil
      if i == 1 then
        tIdx, windowName = itwd.idx1, itwd.name1
      elseif i == 2 then
        tIdx, windowName = itwd.idx2, itwd.name2
      else
        tIdx, windowName = itwd.idx3, itwd.name3
      end

      if tIdx ~= nil and trajectories[tIdx] ~= nil then
        local tr = trajectories[tIdx]
        local isTimeBased = tr.isTimeBased
        -- Get the relevant polyline from the associated trajectory.
        local polyLine = getPolyRef(tr)
        local isSpline = tr.isUseSpline[0]
        local pos = tr.selectedNode
        -- Display the window.
        if editor.beginWindow(windowName, "Trajectory - " .. tostring(tIdx)) then
          -- The trajectory color bar.
          im.ColorEdit3("", tr.col)
          im.tooltip('Select a color for the trajectory.')
          im.Separator()
          -- The nodes list.
          if im.BeginListBox("", im.ImVec2(330, 200), im.WindowFlags_ChildWindow) then
            im.Columns(6, "indTrajListCols", false)
            im.SetColumnWidth(0, 50)
            im.SetColumnWidth(1, 50)
            im.SetColumnWidth(2, 60)
            im.SetColumnWidth(3, 60)
            im.SetColumnWidth(4, 60)
            im.SetColumnWidth(5, 50)
            local numNodes = #polyLine
            for j = 1, numNodes do
              local n = polyLine[j]
              local flag = false
              if j == tr.selectedNode or n.isLocked == true then
                flag = true
              end
              if im.Selectable1("[" .. j .. "] :", flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
                tr.selectedNode = j
              end
              im.NextColumn()
              if isTimeBased == true then
                im.Text(round1(n.t[0]) .. "s")
              else
                im.Text(round1(n.v[0] * 3.6) .. "kph")
              end
              im.NextColumn()
              im.Text("(x: " .. round1(n.x[0]) .. ",")
              im.NextColumn()
              im.Text("y: " .. round1(n.y[0]) .. ",")
              im.NextColumn()
              im.Text("z: " .. round1(n.z[0]) .. ")")
              im.NextColumn()
              if n.isLocked then
                if editor.uiIconImageButton(editor.icons.lock, im.ImVec2(17, 17), colors.lock, nil, nil, 'unlockNode') then
                  if tr.isTimeBased == true then
                    local oldNodes = polyPtr2ValT(polyLine)
                    n.isLocked = false
                    local newNodes = polyPtr2ValT(polyLine)
                    local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                    editor.history:commitAction("Unlock Node", data, nodesUndo, nodesRedo)
                  else
                    local oldNodes = polyPtr2ValV(polyLine)
                    n.isLocked = false
                    local newNodes = polyPtr2ValV(polyLine)
                    local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                    editor.history:commitAction("Unlock Node", data, nodesUndo, nodesRedo)
                  end
                end
                im.tooltip('Unlock the highlighted node, so it can be moved in time/space.')
              end
              im.NextColumn()
              im.Separator()
            end
            im.EndListBox()
          end
          im.Separator()
          -- Set the gap between buttons, depending on which mode the window relates to.
          local gapA, gapB = 10, 15
          if tr.isTimeBased == false then
            gapA, gapB = 0, 1
          end
          -- The row of buttons below the nodes list.
          im.Dummy(im.ImVec2(gapA, 0))
          im.SameLine()
          if #polyLine > 1 then
            if tr.isTimeBased == false then
              im.PushButtonRepeat(true)
              if editor.uiIconImageButton(editor.icons.arrow_upward, im.ImVec2(27, 27), nil, nil, nil, 'increaseVel') then
                polyLine[tr.selectedNode].v = im.FloatPtr(min(consts.maxMPS, polyLine[tr.selectedNode].v[0] + consts.incMPS))
              end
              im.tooltip('Increase the velocity at the highlighted node.')
              im.SameLine()
              im.Dummy(im.ImVec2(gapB, 0))
              im.SameLine()
              im.PushButtonRepeat(true)
              if editor.uiIconImageButton(editor.icons.arrow_downward, im.ImVec2(27, 27), nil, nil, nil, 'decreaseVel') then
                polyLine[tr.selectedNode].v = im.FloatPtr(max(consts.minMPS, polyLine[tr.selectedNode].v[0] - consts.incMPS))
              end
              im.tooltip('Decrease the velocity at the highlighted node.')
              im.SameLine()
              im.Dummy(im.ImVec2(gapB, 0))
              im.SameLine()
            end
            if editor.uiIconImageButton(editor.icons.nodeLast01, im.ImVec2(27, 27), nil, nil, nil, 'addNode') then
              if tr.isTimeBased == true then
                local oldNodes = polyPtr2ValT(polyLine)
                addNode(polyLine, pos, isTimeBased)
                local newNodes = polyPtr2ValT(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Add Node", data, nodesUndo, nodesRedo)
              else
                local oldNodes = polyPtr2ValV(polyLine)
                addNode(polyLine, pos, isTimeBased)
                local newNodes = polyPtr2ValV(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Add Node", data, nodesUndo, nodesRedo)
              end
            end
            im.tooltip('Add a new node directly after the highlighted node.')
            im.SameLine()
            im.Dummy(im.ImVec2(gapB, 0))
            im.SameLine()
            if editor.uiIconImageButton(editor.icons.control_point_duplicate, im.ImVec2(27, 27), nil, nil, nil, 'doubleNodeResolution') then
              if tr.isTimeBased == true then
                local oldNodes = polyPtr2ValT(polyLine)
                local idx, len = 1, #polyLine
                while idx < len do
                  addNode(polyLine, idx, isTimeBased)
                  idx = idx + 2
                end
                local newNodes = polyPtr2ValT(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Double Node Resolution", data, nodesUndo, nodesRedo)
              else
                local oldNodes = polyPtr2ValV(polyLine)
                local idx, len = 1, #polyLine
                while idx < len do
                  addNode(polyLine, idx, isTimeBased)
                  idx = idx + 2
                end
                local newNodes = polyPtr2ValV(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Double Node Resolution", data, nodesUndo, nodesRedo)
              end
            end
            im.tooltip('Double the node resolution for finer-grained detailing.')
            im.SameLine()
          end
          im.Dummy(im.ImVec2(gapB, 0))
          im.SameLine()
          if editor.uiIconImageButton(editor.icons.nodeRemove, im.ImVec2(27, 27), nil, nil, nil, 'deleteNode') then
            if tr.isTimeBased == true then
              local oldNodes = polyPtr2ValT(polyLine)
              if #polyLine > 2 then
                table.remove(polyLine, tr.selectedNode)
                tr.selectedNode = min(max(1, tr.selectedNode), #polyLine)
              else
                tr = nil
              end
              local newNodes = polyPtr2ValT(polyLine)
              local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
              editor.history:commitAction("Remove Node", data, nodesUndo, nodesRedo)
            else
              local oldNodes = polyPtr2ValV(polyLine)
              if #polyLine > 2 then
                table.remove(polyLine, tr.selectedNode)
                tr.selectedNode = min(max(1, tr.selectedNode), #polyLine)
              else
                tr = nil
              end
              local newNodes = polyPtr2ValV(polyLine)
              local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
              editor.history:commitAction("Remove Node", data, nodesUndo, nodesRedo)
            end
            return
          end
          im.tooltip('Remove the highlighted node.')
          im.SameLine()
          im.Dummy(im.ImVec2(gapB, 0))
          im.SameLine()
          pos = tr.selectedNode
          if polyLine[pos].isLocked == false then
            if editor.uiIconImageButton(editor.icons.lock, im.ImVec2(27, 27), colors.lock, nil, nil, 'lockNode') then
              if tr.isTimeBased == true then
                local oldNodes = polyPtr2ValT(polyLine)
                polyLine[pos].isLocked = true
                local newNodes = polyPtr2ValT(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Lock Node", data, nodesUndo, nodesRedo)
              else
                local oldNodes = polyPtr2ValV(polyLine)
                polyLine[pos].isLocked = true
                local newNodes = polyPtr2ValV(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Lock Node", data, nodesUndo, nodesRedo)
              end
            end
            im.tooltip('Lock the highlighted node, so that its values will not change.')
          else
            if editor.uiIconImageButton(editor.icons.lock_open, im.ImVec2(27, 27), colors.lock, nil, nil, 'unlockNode') then
              if tr.isTimeBased == true then
                local oldNodes = polyPtr2ValT(polyLine)
                polyLine[pos].isLocked = false
                local newNodes = polyPtr2ValT(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Unlock Node", data, nodesUndo, nodesRedo)
              else
                local oldNodes = polyPtr2ValV(polyLine)
                polyLine[pos].isLocked = false
                local newNodes = polyPtr2ValV(polyLine)
                local data = { old = oldNodes, new = newNodes, isSpline = isSpline, tIdx = tIdx }
                editor.history:commitAction("Unlock Node", data, nodesUndo, nodesRedo)
              end
            end
            im.tooltip('Unlock the highlighted node, so it can be moved in time/space.')
          end
          im.SameLine()
          im.Dummy(im.ImVec2(gapB, 0))
          im.SameLine()
          if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, im.ImVec2(24, 24), nil, nil, nil, 'revealTrajectory') then
            moveCam2Traj(tr.polyLine)
          end
          im.tooltip('Reveal this trajectory on the map, from a top-down view.')
          im.SameLine()
          im.Dummy(im.ImVec2(gapB, 0))
          im.SameLine()
          if editor.uiIconImageButton(editor.icons.floppyDisk, im.ImVec2(23, 23), nil, nil, nil, 'exportTrajectory') then
            export(polyLine, tr)
          end
          im.tooltip('Export this trajectory to file.')
          im.Separator()

          -- Display the mode, interval range, attach status, and attachment buttons, for this trajectory
          if isTimeBased == true then
            im.Text("Mode: [Time-Based]")
            im.Text('Interval: [' .. tostring(round2(polyLine[1].t[0])) .. ', ' .. tostring(round2(polyLine[#polyLine].t[0])) .. ']')
          else
            im.Text("Mode: [Velocity-Based]")
            im.Text("Interval:")
            im.SameLine()
            if tr.vModeTStart == nil then tr.vModeTStart = im.FloatPtr(0.0) end
            if tr.vModeTEnd == nil then tr.vModeTEnd = im.FloatPtr(60.0) end
            local oldStart, oldEnd = tr.vModeTStart[0], tr.vModeTEnd[0]
            im.PushItemWidth(100)
            im.InputFloat("start", tr.vModeTStart, 1, 0.0)
            im.tooltip('The start time at which to execute this trajectory.')
            im.PopItemWidth()
            im.SameLine()
            im.PushItemWidth(100)
            im.InputFloat("end", tr.vModeTEnd, 1, 60.0)
            im.tooltip('The end time at which to execute this trajectory.')
            im.PopItemWidth()
            if tr.vModeTStart[0] < 0.0 then
              tr.vModeTStart = im.FloatPtr(0.0)
            end
            if tr.vModeTStart[0] >= tr.vModeTEnd[0] then
              tr.vModeTStart, tr.vModeTEnd = im.FloatPtr(oldStart), im.FloatPtr(oldEnd)
            end
          end
          if tr.vehicle == nil or tr.vehicle == "" then
            im.Text('Status: [not attached to vehicle]')
            im.SameLine()
            if editor.uiIconImageButton(editor.icons.jointUnlocked, im.ImVec2(26, 26), nil, nil, nil, 'attachTrajectoryToVehicle') then
              local oldTraj = ptr2ValTraj(tr)
              local veh = sceneVehicles[vehWinData.selectedVeh]
              local selectedVehicleString = veh.string
              detachTrajectory(selectedVehicleString)
              tr.vehicle, tr.vid, tr.jBeam = selectedVehicleString, veh.vid, veh.jBeam
              local newTraj = ptr2ValTraj(tr)
              local data = { old = oldTraj, new = newTraj, tIdx = tIdx }
              editor.history:commitAction("Attach trajectory to vehicle", data, indTrajWinUndo, indTrajWinRedo)
            end
            im.tooltip('Attach this trajectory to the selected vehicle.')
          else
            im.Text('Status: [' .. tr.vehicle .. ']')
            im.SameLine()
            if editor.uiIconImageButton(editor.icons.jointLocked, im.ImVec2(22, 22), nil, nil, nil, 'detachTrajectoryFromVehicle') then
              local oldTraj = ptr2ValTraj(tr)
              tr.vehicle, tr.vid, tr.jBeam = "", nil, nil
              local newTraj = ptr2ValTraj(tr)
              local data = { old = oldTraj, new = newTraj, tIdx = tIdx }
              editor.history:commitAction("Detach trajectory from vehicle", data, indTrajWinUndo, indTrajWinRedo)
            end
            im.tooltip('Detach this trajectory from its linked vehicle.')
            im.SameLine()
            if editor.uiIconImageButton(editor.icons.car, im.ImVec2(24, 24), nil, nil, nil, 'goToLinkedVehicle') then
              core_camera.setByName(0, "orbit", false)
              be:enterVehicle(0, scenetree.findObject(tr.vid))
            end
            im.tooltip("Move camera to this trajectory's linked vehicle.")
          end
          im.Separator()
          -- The row for manual trajectory velocity setting dialog.
          if editor.uiIconImageButton(editor.icons.network_check, im.ImVec2(26, 26), nil, nil, nil, 'setTrajectoryVelocity') then
            local oldTraj = ptr2ValTraj(tr)
            local velMps = tr.inputVelocity[0] * 0.277778
            if isTimeBased == true then
              polyLine = setFixedVelPtr(polyLine, velMps)
            else
              local len = #polyLine
              for i = 1, len do
                polyLine[i].v = im.FloatPtr(velMps)
              end
            end
            local newTraj = ptr2ValTraj(tr)
            local data = { old = oldTraj, new = newTraj, tIdx = tIdx }
            editor.history:commitAction("Set velocity.", data, indTrajWinUndo, indTrajWinRedo)
          end
          im.tooltip('Assign the set velocity to the trajectory.')
          im.SameLine()
          im.PushItemWidth(110)
          im.InputFloat(" Velocity (kph)", tr.inputVelocity, 10.0, 1.0)
          im.tooltip('Enter a velocity to be set along the full length of this trajectory.')
          im.PopItemWidth()
          if tr.inputVelocity[0] < 0.1 then
            tr.inputVelocity = im.FloatPtr(0.1)
          elseif tr.inputVelocity[0] > 180.0 then
            tr.inputVelocity = im.FloatPtr(180.0)
          end
          im.Separator()
          -- The row for spline fitting dialog.
          if im.Checkbox("Fit Spline", tr.isUseSpline) then
            if tr.isUseSpline[0] == true then fitSplineToTraj(tr) end
            tr.selectedNode = 1
          end
          im.tooltip('Switch between spline/polyline representation.')
          if tr.isUseSpline[0] == true then
            im.SameLine()
            local oldVal = tr.splineSpacing[0]
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            im.PushItemWidth(245)
            im.SliderInt("##2", tr.splineSpacing, 1, 20)
            im.PopItemWidth()
            im.PopStyleVar()
            if tr.splineSpacing[0] ~= oldVal then
              fitSplineToTraj(tr)
              tr.selectedNode = 1
            end
          end
          im.Separator()
          im.Columns(3, "indTrajCheckBoxCols", false)
          im.SetColumnWidth(0, 115)
          im.SetColumnWidth(1, 115)
          im.SetColumnWidth(2, 115)
          -- The top row of checkboxes.
          im.Checkbox("Trajectory", tr.isDisplay)
          im.tooltip('Switch the visual display of this trajectory on/off.')
          im.NextColumn()
          im.Checkbox("Nodes", tr.isMarkNodes)
          im.tooltip('Includes/Removes the trajectory nodes on the display.')
          im.NextColumn()
          im.Checkbox("Velocities", tr.isMarkVelocities)
          im.tooltip('Includes/Removes the line segment velocities on the display.')
          im.NextColumn()
          im.Separator()
          -- The bottom row of checkboxes.
          im.Checkbox("Rigid Path", tr.isUseRigidTranslation)
          im.tooltip('Treats the full trajectory as a rigid body when moving it on the map.')
          if isTimeBased == true then
            im.NextColumn()
            im.Checkbox("AI Assistant", tr.isExternalForce)
            im.tooltip('Switches AI assistance on/off. This helps to keep the vehicle on course.')
            im.NextColumn()
            im.Checkbox("Hold Velocity", tr.isHoldVelocity)
            im.tooltip('Maintains the average velocities of each section, as nodes are moved on the map.')
            im.NextColumn()
          else
            im.Dummy(im.ImVec2(25, 25))
            im.NextColumn()
            im.Dummy(im.ImVec2(25, 25))
            im.NextColumn()
          end
          -- A slider for setting the attraction force field, for when editing the trajectory on the map.
          im.Columns(2, "indTrajForceSliderCols", false)
          im.SetColumnWidth(0, 340)
          im.SetColumnWidth(1, 1)
          im.Separator()
          if tr.isUseRigidTranslation[0] == false then
            im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
            im.PushItemWidth(340)
            im.SliderFloat("", tr.fieldRange, 0.1, 200.0, "Field = %.3f")
            im.tooltip('Sets the attraction force field while editing this trajectory on the map.')
            im.PopItemWidth()
            im.PopStyleVar()
          end
        else
          -- The individual trajectory window was closed, so we need to remove the index from the window state.
          if i == 1 then
            itwd.idx1 = nil
          elseif i == 2 then
            itwd.idx2 = nil
          else
            itwd.idx3 = nil
          end
        end
        editor.endWindow()
      end
    end
  end

  -- Display the Camera trajectory window.
  local cwd = camWinData
  if editor.isWindowVisible(cwd.name) and numVehicles > 0 and drawWinData.isDrawIn == false and twd.isExecuting == false then
    local isChange = false
    local isDisabled = false
    local oldVals = camPathPtr2Val()

    if editor.beginWindow(camWinData.name, "Camera Trajectory") then
      -- The camera nodes list.
      im.Separator()
      if im.BeginListBox("", im.ImVec2(820, 110), im.WindowFlags_ChildWindow) then
        im.Columns(6, "camWinColumns", false)
        im.SetColumnWidth(0, 60)
        im.SetColumnWidth(1, 220)
        im.SetColumnWidth(2, 260)
        im.SetColumnWidth(3, 120)
        im.SetColumnWidth(4, 120)
        im.SetColumnWidth(5, 40)
        local nodes = cwd.nodes
        local ctr, numNodes = 1, #nodes
        for j = 1, numNodes do
          local n = nodes[j]
          local flag = false
          if j == cwd.selectedNode then flag = true end
          if im.Selectable1("[" .. j .. "] :", flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then cwd.selectedNode = j end
          im.NextColumn()
          local tOld = n.t[0]
          if im.InputFloat("Time##" .. tostring(ctr), n.t, 0.1, 1.0) then
            isChange = true
          end
          im.tooltip('Change the time when the camera should arrive at this node.')
          if (j > 1 and n.t[0] <= nodes[j - 1].t[0]) or (j < numNodes and n.t[0] >= nodes[j + 1].t[0]) then n.t = im.FloatPtr(tOld) end
          if n.t[0] < 0.0 then n.t = im.FloatPtr(0.0) end
          im.NextColumn()
          if im.InputFloat("Smoothness##" .. tostring(ctr + 1), n.smoothness, 0.1, 0.1) then
            isChange = true
          end
          im.tooltip('Change the smoothness value of this node, in [0, 1].')
          if n.smoothness[0] < 0.0 then n.smoothness = im.FloatPtr(0.0) end
          if n.smoothness[0] > 1.0 then n.smoothness = im.FloatPtr(1.0) end
          im.NextColumn()
          im.Checkbox("Moving Start##" .. tostring(ctr + 2), n.movingStart)
          im.tooltip('Set whether or not to allow continuous movement on approach to this node.')
          im.NextColumn()
          im.Checkbox("Moving End##" .. tostring(ctr + 3), n.movingEnd)
          im.tooltip('Set whether or not to allow continuous movement on departure from this node.')
          im.NextColumn()
          if n.isLocked then
            if editor.uiIconImageButton(editor.icons.lock, im.ImVec2(17, 17), colors.lock, nil, nil, 'unlockNode') then
              n.isLocked = false
              isChange = true
            end
            im.tooltip('Unlock the highlighted node, so it can be moved in time/space.')
          end
          im.NextColumn()
          im.Separator()
          ctr = ctr + 4
        end
        im.EndListBox()
      end
      im.Separator()

      im.Columns(8, "camWinButtonCols", false)
      im.SetColumnWidth(0, 40)
      im.SetColumnWidth(1, 40)
      im.SetColumnWidth(2, 40)
      im.SetColumnWidth(3, 40)
      im.SetColumnWidth(4, 40)
      im.SetColumnWidth(5, 200)
      im.SetColumnWidth(6, 260)
      im.SetColumnWidth(7, 200)
      local tNow = toolWinData.t[0]
      local nodes, numNodes = cwd.nodes, #cwd.nodes

      isDisabled = numNodes > 1 and tNow > nodes[numNodes].t[0]
      if isDisabled then im.BeginDisabled() end
      if editor.uiIconImageButton(editor.icons.nodeAddFirst01, im.ImVec2(26, 26), nil, nil, nil, 'addCamNode') then
        local oldData = serializeCamData()
        local pos, rot = core_camera.getPosition(), core_camera.getQuat()
        rot:normalize()
        nodes[numNodes + 1] = {
          x = im.FloatPtr(pos.x), y = im.FloatPtr(pos.y), z = im.FloatPtr(pos.z),
          qx = rot.x, qy = rot.y, qz = rot.z, qw = rot.w,
          t = im.FloatPtr(tNow),
          smoothness = im.FloatPtr(0.5),
          movingStart = im.BoolPtr(true), movingEnd = im.BoolPtr(true),
          isLocked = false }
        local newData = serializeCamData()
        local data = { old = oldData, new = newData }
        editor.history:commitAction("Adjust camera node time.", data, camWinUndo, camWinRedo)
      end
      im.tooltip('Adds a camera trajectory node at the current camera position and current time.')
      if isDisabled then im.EndDisabled() end
      im.NextColumn()
      isDisabled = numNodes < 1
      if isDisabled then im.BeginDisabled() end
      if editor.uiIconImageButton(editor.icons.nodeRemove, im.ImVec2(26, 26), nil, nil, nil, 'removeCamNode') then
        local oldData = serializeCamData()
        table.remove(nodes, cwd.selectedNode)
        cwd.selectedNode = min(max(1, cwd.selectedNode), #nodes)
        local newData = serializeCamData()
        local data = { old = oldData, new = newData }
        editor.history:commitAction("Adjust camera node time.", data, camWinUndo, camWinRedo)
      end
      im.tooltip('Removes the selected camera node.')
      if isDisabled then im.EndDisabled() end
      im.NextColumn()
      isDisabled = numTrajectories < 1 or trajWinData.selectedTraj == nil
      if isDisabled then im.BeginDisabled() end
      if editor.uiIconImageButton(editor.icons.touch_app, im.ImVec2(25, 25), nil, nil, nil, 'camAutoGenerate') then
        local tr = trajectories[trajWinData.selectedTraj]
        local poly = getPolyRef(tr)
        local len = #poly
        table.clear(cwd.nodes)
        local rot = quatFromDir(vec3(0, 0, -1))
        rot:normalize()
        for i = 1, len do
          local n = poly[i]
          cwd.nodes[i] = {
            x = im.FloatPtr(n.x[0]), y = im.FloatPtr(n.y[0]), z = im.FloatPtr(n.z[0] + 30.0),
            qx = rot.x, qy = rot.y, qz = rot.z, qw = rot.w,
            t = im.FloatPtr(n.t[0]),
            smoothness = im.FloatPtr(0.5),
            movingStart = im.BoolPtr(true), movingEnd = im.BoolPtr(true),
            isLocked = false }
        end
      end
      im.tooltip('Auto generate a simple top-down camera path along the currently-selected trajectory.')
      if isDisabled then im.EndDisabled() end
      im.NextColumn()
      if editor.uiIconImageButton(editor.icons.cameraFocusTopDown, im.ImVec2(24, 24), nil, nil, nil, 'camTrajReveal') then
        moveCam2Traj(cwd.nodes)
      end
      im.tooltip('Shows a top-down view of the full camera trajectory.')
      im.NextColumn()
      if #cwd.nodes > 0 and cwd.selectedNode > 0 and cwd.selectedNode <= #cwd.nodes then
        local nSel = nodes[cwd.selectedNode]
        if nSel.isLocked == false then
          if editor.uiIconImageButton(editor.icons.lock, im.ImVec2(27, 27), colors.lock, nil, nil, 'lockNode') then
            nSel.isLocked = true
          end
          im.tooltip('Lock the highlighted node, so that its values will not change.')
        else
          if editor.uiIconImageButton(editor.icons.lock_open, im.ImVec2(27, 27), colors.lock, nil, nil, 'unlockNode') then
            nSel.isLocked = false
          end
          im.tooltip('Unlock the highlighted node, so it can be moved in time/space.')
        end
      end
      im.NextColumn()
      im.Checkbox("Display", cwd.isDisplay)
      im.tooltip('Toggles the camera trajectory display on the map, when editing.')
      im.SameLine()
      im.Checkbox("On Execute", cwd.isOnExecute)
      im.tooltip('Toggles whether the camera path will be used during execution.')
      im.NextColumn()
      im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
      im.PushItemWidth(200)
      im.SliderFloat("", cwd.fieldRange, 0.1, 200.0, "Field = %.3f")
      im.tooltip('Sets the attraction force field while editing this camera path.')
      im.PopItemWidth()
      im.PopStyleVar()
      im.NextColumn()
      im.ColorEdit3("", cwd.col)
      im.tooltip('Selects a color for the camera trajectory.')
      im.NextColumn()
    else
      winStates.camWin.buttonAlpha = 1
    end
    editor.endWindow()

    if isChange == true then
      local newVals = camPathPtr2Val()
      local data = { old = oldVals, new = newVals }
      editor.history:commitAction("Camera path value changed.", data, camNodesUndo, camNodesRedo)
    end
  end

  -- Manage the Draw-In Window.
  local dwd = drawWinData
  if dwd.isDrawIn == true and twd.isExecuting == false then
    if editor.beginWindow(dwd.name, "Draw Trajectory") then
      im.ColorEdit3("", dwd.drawCol)
      im.tooltip('Select a color for the trajectory.')
      im.Separator()
      if im.BeginListBox("", im.ImVec2(215, 250), im.WindowFlags_ChildWindow) then
        local numdrawNodes = #dwd.drawNodes
        for i = 1, numdrawNodes do
          im.Columns(4, "columns4", false)
          im.SetColumnWidth(0, 30)
          im.SetColumnWidth(1, 60)
          im.SetColumnWidth(2, 60)
          im.SetColumnWidth(3, 60)
          im.Text("[" .. i .. "]:")
          im.NextColumn()
          local n = dwd.drawNodes[i]
          im.Text("(x: " .. round1(n.x) .. ",")
          im.NextColumn()
          im.Text("y: " .. round1(n.y) .. ",")
          im.NextColumn()
          im.Text("z: " .. round1(n.z) .. ")")
          im.NextColumn()
          im.Separator()
        end
        im.tooltip('Finish drawing trajectory, and add to the trajectory list.')
        im.EndListBox()
      end
      im.Separator()
      im.RadioButton2("T-Based", dwd.mode, im.Int(1))
      im.SameLine()
      im.RadioButton2("V-Based", dwd.mode, im.Int(2))
      im.Separator()
      if #dwd.drawNodes > 1 then
        if editor.uiIconImageButton(editor.icons.check, im.ImVec2(26, 26), nil, nil, nil, 'finishDrawInTrajectory') then
          local tIdx = "drawn_" .. tostring(getNextUniqueId())
          local isTimeBased = true
          if dwd.mode[0] == 2 then isTimeBased = false end
          if isTimeBased == true then                                                       -- For time-based mode, set default times to ensure a constant speed of 30 kph.
            dwd.drawNodes = setFixedVelValT(dwd.drawNodes, 8.33333333)
          else                                                                              -- For speed-based mode, set 30 kph directly as the max speed for all nodes.
            local len = #dwd.drawNodes
            for i = 1, len do
              dwd.drawNodes[i].v = 8.33333333
              dwd.drawNodes[i] = val2PtrV(dwd.drawNodes[i])
            end
          end
          trajectories[tIdx] = createTrajectory(dwd.drawNodes, nil, dwd.drawCol, nil, isTimeBased)
          dwd.isDrawIn = false
          dwd.selectedNode = im.IntPtr(0)
          dwd.drawNodes = {}
          assignTrajWin(tIdx)
        end
        im.tooltip('Create trajectory and finish drawing.')
        im.SameLine()
        im.AlignTextToFramePadding()
      end
      im.Text("Draw path then click to finish.")
    else                                                                                    -- 'Draw-In Window' was closed, so abandon the drawing and return to normal UI mode.
      dwd.isDrawIn, dwd.selectedNode, dwd.drawNodes = false, im.IntPtr(0), {}
    end
    editor.endWindow()
  end

  -- Handle the trajectory displaying and interaction (this is mode-specific).
  if dwd.isDrawIn == true then                                                              -- When in 'Draw In' mode.
    handleMouseDrawIn()
    drawTrajectories()
  elseif not (twd.isExecuting == true and twd.isDispInExe[0] == false) then                 -- When in 'Edit' mode.
    handleMouseSelection()
    handleMouseDragEditing()
    handleCamGizmo()
    drawTrajectories()
  end
end

-- Called when the ScriptAI Editor icon is pressed.
local function onActivate()
  editor.clearObjectSelection()
  editor.showWindow(toolWinData.name)
end

-- Called when the ScriptAI Editor is exited.
local function onDeactivate()
  editor.hideWindow(toolWinData.name)
  editor.hideWindow(vehWinData.name)
  editor.hideWindow(trajWinData.name)
  editor.hideWindow(indTrajWinData.name1)
  editor.hideWindow(indTrajWinData.name2)
  editor.hideWindow(indTrajWinData.name3)
  editor.hideWindow(camWinData.name)
  editor.hideWindow(drawWinData.name)
  winStates.vehWin.buttonAlpha, winStates.trajWin.buttonAlpha, winStates.camWin.buttonAlpha = 1, 1, 1
end

-- Called upon world editor initialization.
local function onEditorInitialized()
  editor.editModes.scriptAIEditMode = {
    displayName = editModeName,
    onUpdate = nop,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    icon = editor.icons.BNGMicrochip,
    iconTooltip = "ScriptAI Editor",
    auxShortcuts = {},
    hideObjectIcons = true }
  editor.registerWindow(toolWinData.name, toolWinData.winSize)
  editor.registerWindow(vehWinData.name, vehWinData.winSize)
  editor.registerWindow(trajWinData.name, trajWinData.winSize)
  editor.registerWindow(indTrajWinData.name1, indTrajWinData.winSize)                      -- Note: we allow up to three individual trajectory windows.
  editor.registerWindow(indTrajWinData.name2, indTrajWinData.winSize)
  editor.registerWindow(indTrajWinData.name3, indTrajWinData.winSize)
  editor.registerWindow(camWinData.name, camWinData.winSize)
  editor.registerWindow(drawWinData.name, drawWinData.winSize)

  if winStates.vehWin.visible then editor.showWindow(vehWinData.name) end
  if winStates.trajWin.visible then editor.showWindow(trajWinData.name) end
  if winStates.camWin.visible then editor.showWindow(camWinData.name) end
end

-- Triggering when a recording is stopped.
local function onVehicleSubmitRecording(vid, data)
  if not editor.editMode or editor.editMode.displayName ~= editModeName then return end

  detachTrajectory(findVehDataByVid(vid))
  local numTrajectories = 0
  for k, tr in pairs(trajectories) do
    numTrajectories = numTrajectories + 1
  end
  local tIdx = "recorded_" .. tostring(getNextUniqueId())
  local isTimeBased = true
  if data.path[1].v ~= nil then isTimeBased = false end
  trajectories[tIdx] = createTrajectory(data.path, vid, nil, nil, isTimeBased)
  assignTrajWin(tIdx)
end

-- Triggered when a vehicle is replaced.
local function onVehicleReplaced(vid)
  local vString, jBeam = "", nil
  for id, veh in activeVehiclesIterator() do
    if vid == id then
      local vehName = veh:getName()
      jBeam = veh.JBeam
      vString= tostring(vid .. ": " .. vehName .. " - " .. jBeam)
    end
  end
  for k, tr in pairs(trajectories) do
    if tr.vid == vid then
      tr.vehicle, tr.jBeam = vString, jBeam
    end
  end
end

-- Serialization function.
local function onSerialize()
  return { d = lpack.encode{
    trajectories = serializeTrajData(),
    camData = serializeCamData(),
    t = toolWinData.t[0],
    selectedVeh = vehWinData.selectedVeh,
    uniqueTrajectoryId = uniqueId,
    selectedTraj = trajWinData.selectedTraj,
    isVehWinVisible = false,
    isTrajWinVisible = false,
    isCamWinVisible = false,
    vehWinButtonAlpha = winStates.vehWin.buttonAlpha,
    trajWinButtonAlpha = winStates.trajWin.buttonAlpha,
    camWinButtonAlpha = winStates.camWin.buttonAlpha } }
end

-- Deserialization function.
local function onDeserialized(dataIn)
  local data = lpack.decode(dataIn.d)
  deserializeTrajData(data.trajectories)
  deserializeCamData(data.camData)
  toolWinData.t = im.FloatPtr(data.t)
  vehWinData.selectedVeh = data.selectedVeh
  uniqueId = data.uniqueTrajectoryId
  winStates.vehWin.visible = data.isVehWinVisible
  winStates.trajWin.visible = data.isTrajWinVisible
  winStates.camWin.visible = data.isCamWinVisible
  winStates.vehWin.buttonAlpha = data.vehWinButtonAlpha
  winStates.trajWin.buttonAlpha = data.trajWinButtonAlpha
  winStates.camWin.buttonAlpha = data.camWinButtonAlpha
end

-- Public interface.

-- Functions triggered by hooks.
M.onEditorGui                               = onEditorGui
M.onEditorInitialized                       = onEditorInitialized
M.onVehicleSubmitRecording                  = onVehicleSubmitRecording
M.onVehicleReplaced                         = onVehicleReplaced
M.onSerialize                               = onSerialize
M.onDeserialized                            = onDeserialized

return M