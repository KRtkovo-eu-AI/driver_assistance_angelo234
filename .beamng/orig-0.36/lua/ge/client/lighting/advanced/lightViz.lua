-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local curVisualizeModeName = nil
local function onEnabledVisualization(newVisualizeModeName)
  if curVisualizeModeName ~= newVisualizeModeName then
    local curVisualizeMode = curVisualizeModeName and scenetree.findObject(curVisualizeModeName)
    if curVisualizeMode then
        curVisualizeMode:disable()
    end
    curVisualizeModeName = newVisualizeModeName
  end
end

local al_DepthVisualizeState = scenetree.findObject("AL_DepthVisualizeState")
if not al_DepthVisualizeState then
  al_DepthVisualizeState = createObject("GFXStateBlockData")
  al_DepthVisualizeState.zDefined = true
  al_DepthVisualizeState.zEnable = false
  al_DepthVisualizeState.zWriteEnable = false
  al_DepthVisualizeState.samplersDefined = true
  al_DepthVisualizeState:setField("samplerStates", 0, "SamplerClampPoint") -- prepass
  al_DepthVisualizeState:setField("samplerStates", 1, "SamplerClampPoint") -- prepass depth
  al_DepthVisualizeState:setField("samplerStates", 2, "SamplerClampLinear") -- viz color lookup
  al_DepthVisualizeState:registerObject("AL_DepthVisualizeState")
end

local al_DefaultVisualizeState = scenetree.findObject("AL_DefaultVisualizeState")
if not al_DefaultVisualizeState then
  al_DefaultVisualizeState = createObject("GFXStateBlockData")
  al_DefaultVisualizeState.blendDefined = true
  al_DefaultVisualizeState.blendEnable = true
  al_DefaultVisualizeState:setField("blendSrc", 0, "GFXBlendSrcAlpha")
  al_DefaultVisualizeState:setField("blendDest", 0, "GFXBlendInvSrcAlpha")
  al_DefaultVisualizeState.zDefined = true
  al_DefaultVisualizeState.zEnable = false
  al_DefaultVisualizeState.zWriteEnable = false
  al_DefaultVisualizeState.samplersDefined = true
  al_DefaultVisualizeState:setField("samplerStates", 0, "SamplerClampPoint") --prepass
  al_DefaultVisualizeState:setField("samplerStates", 1, "SamplerClampLinear") --depthviz
  al_DefaultVisualizeState:registerObject("AL_DefaultVisualizeState")
end

local al_DepthVisualizeShader = scenetree.findObject("AL_DepthVisualizeShader")
if not al_DepthVisualizeShader then
  al_DepthVisualizeShader = createObject("ShaderData")
  al_DepthVisualizeShader.DXVertexShaderFile = "shaders/common/lighting/advanced/dbgDepthVisualizeP.hlsl"
  al_DepthVisualizeShader.DXPixelShaderFile  = "shaders/common/lighting/advanced/dbgDepthVisualizeP.hlsl"
  al_DepthVisualizeShader:setField("samplerNames", 0, "prepassBuffer")
  al_DepthVisualizeShader:setField("samplerNames", 1, "depthViz")
  al_DepthVisualizeShader.pixVersion = 5.0;
  al_DepthVisualizeShader:registerObject("AL_DepthVisualizeShader")
end

local al_DepthVisualizeCallbacks = {}
al_DepthVisualizeCallbacks.onEnabled = function()
  onEnabledVisualization("AL_DepthVisualize")
  return true
end
rawset(_G, "AL_DepthVisualizeCallbacks", al_DepthVisualizeCallbacks)

local al_DepthVisualize = scenetree.findObject("AL_DepthVisualize")
if not al_DepthVisualize then
  al_DepthVisualize = createObject("PostEffect")
  al_DepthVisualize:setField("shader", 0, "AL_DepthVisualizeShader")
  al_DepthVisualize:setField("stateBlock", 0, "AL_DepthVisualizeState")
  al_DepthVisualize:setField("texture", 0, "#prepass[RT0]")
  al_DepthVisualize:setField("texture", 1, "#prepass[Depth]")
  al_DepthVisualize:setField("texture", 2, "lua/ge/client/lighting/advanced/depthviz.png")
  al_DepthVisualize:setField("target", 0, "$backBuffer")
  al_DepthVisualize.renderPriority = 9999
  al_DepthVisualize:registerObject("AL_DepthVisualize")
end

local al_NormalsVisualizeShader = scenetree.findObject("AL_NormalsVisualizeShader")
if not al_NormalsVisualizeShader then
  al_NormalsVisualizeShader = createObject("ShaderData")
  al_NormalsVisualizeShader.DXVertexShaderFile  = "shaders/common/lighting/advanced/dbgNormalVisualizeP.hlsl"
  al_NormalsVisualizeShader.DXPixelShaderFile   = "shaders/common/lighting/advanced/dbgNormalVisualizeP.hlsl"
  al_NormalsVisualizeShader:setField("samplerNames", 0, "prepassTex")
  al_NormalsVisualizeShader.pixVersion = 5.0;
  al_NormalsVisualizeShader:registerObject("AL_NormalsVisualizeShader")
end

local aL_NormalsVisualizeCallbacks = {}
 aL_NormalsVisualizeCallbacks.onEnabled = function()
  onEnabledVisualization("AL_NormalsVisualize");
  return true
end
rawset(_G, "AL_NormalsVisualize", aL_NormalsVisualizeCallbacks)

local al_NormalsVisualize = scenetree.findObject("AL_NormalsVisualize")
if not al_NormalsVisualize then
  al_NormalsVisualize = createObject("PostEffect")
  al_NormalsVisualize:setField("shader", 0, "AL_NormalsVisualizeShader")
  al_NormalsVisualize:setField("stateBlock", 0, "AL_DefaultVisualizeState")
  al_NormalsVisualize:setField("texture", 0, "#prepass[RT0]")
  al_NormalsVisualize:setField("texture", 1, "#prepass[Depth]")
  al_NormalsVisualize:setField("target", 0, "$backBuffer")
  al_NormalsVisualize.renderPriority = 9999
  al_NormalsVisualize:registerObject("AL_NormalsVisualize")
end

local al_LightColorVisualizeShader = scenetree.findObject("AL_LightColorVisualizeShader")
if not al_LightColorVisualizeShader then
  al_LightColorVisualizeShader = createObject("ShaderData")
  al_LightColorVisualizeShader.DXVertexShaderFile  = "shaders/common/lighting/advanced/dbgLightColorVisualizeP.hlsl"
  al_LightColorVisualizeShader.DXPixelShaderFile   = "shaders/common/lighting/advanced/dbgLightColorVisualizeP.hlsl"
  al_LightColorVisualizeShader:setField("samplerNames", 0, "lightInfoBuffer")
  al_LightColorVisualizeShader.pixVersion = 5.0;
  al_LightColorVisualizeShader:registerObject("AL_LightColorVisualizeShader")
end

local aL_LightColorVisualizeCallbacks = {}
aL_LightColorVisualizeCallbacks.onEnabled = function()
  onEnabledVisualization("AL_LightColorVisualize");
  return true
end
rawset(_G, "AL_LightColorVisualizeCallbacks", aL_LightColorVisualizeCallbacks)

local al_LightColorVisualize = scenetree.findObject("AL_LightColorVisualize")
if not al_LightColorVisualize then
  al_LightColorVisualize = createObject("PostEffect")
  al_LightColorVisualize:setField("shader", 0, "AL_LightColorVisualizeShader")
  al_LightColorVisualize:setField("stateBlock", 0, "AL_DefaultVisualizeState")
  al_LightColorVisualize:setField("texture", 0, "#lightinfo")
  al_LightColorVisualize:setField("target", 0, "$backBuffer")
  al_LightColorVisualize.renderPriority = 9999
  al_LightColorVisualize:registerObject("AL_LightColorVisualize")
end

local al_LightSpecularVisualizeShader = scenetree.findObject("AL_LightSpecularVisualizeShader")
if not al_LightSpecularVisualizeShader then
  al_LightSpecularVisualizeShader = createObject("ShaderData")
  al_LightSpecularVisualizeShader.DXVertexShaderFile  = "shaders/common/lighting/advanced/dbgLightSpecularVisualizeP.hlsl"
  al_LightSpecularVisualizeShader.DXPixelShaderFile   = "shaders/common/lighting/advanced/dbgLightSpecularVisualizeP.hlsl"
  al_LightSpecularVisualizeShader:setField("samplerNames", 0, "lightInfoBuffer")
  al_LightSpecularVisualizeShader.pixVersion = 5.0;
  al_LightSpecularVisualizeShader:registerObject("AL_LightSpecularVisualizeShader")
end

local aL_LightSpecularVisualizeCallbacks = {}
aL_LightSpecularVisualizeCallbacks.onEnabled = function()
  onEnabledVisualization("AL_LightSpecularVisualize");
  return true
end
rawset(_G, "AL_LightSpecularVisualizeCallbacks", aL_LightSpecularVisualizeCallbacks)

local al_LightSpecularVisualize = scenetree.findObject("AL_LightSpecularVisualize")
if not al_LightSpecularVisualize then
  al_LightSpecularVisualize = createObject("PostEffect")
  al_LightSpecularVisualize:setField("shader", 0, "AL_LightSpecularVisualizeShader")
  al_LightSpecularVisualize:setField("stateBlock", 0, "AL_DefaultVisualizeState")
  al_LightSpecularVisualize:setField("texture", 0, "#lightinfo")
  al_LightSpecularVisualize:setField("target", 0, "$backBuffer")
  al_LightSpecularVisualize.renderPriority = 9999
  al_LightSpecularVisualize:registerObject("AL_LightSpecularVisualize")
end

local annotationVisualizeShader = scenetree.findObject("AnnotationVisualizeShader")
if not annotationVisualizeShader then
  annotationVisualizeShader = createObject("ShaderData")
  annotationVisualizeShader.DXVertexShaderFile  = "shaders/common/postFx/annotationViz.hlsl"
  annotationVisualizeShader.DXPixelShaderFile   = "shaders/common/postFx/annotationViz.hlsl"
  annotationVisualizeShader:setField("samplerNames", 0, "AnnotationBuffer")
  annotationVisualizeShader:setField("samplerNames", 1, "warningTex")
  annotationVisualizeShader.pixVersion = 5.0;
  annotationVisualizeShader:registerObject("AnnotationVisualizeShader")
end

local annotationVisualizeCallbacks = {}
annotationVisualizeCallbacks.onEnabled = function()
  if Engine.Annotation then
    onEnabledVisualization("AnnotationVisualize");
    Engine.Annotation.enable(true)
  end
  return true
end

annotationVisualizeCallbacks.onDisabled = function()
  if Engine.Annotation then
    Engine.Annotation.enable(false)
  end
end
rawset(_G, "AnnotationVisualizeCallbacks", annotationVisualizeCallbacks)

local annotationVisualize = scenetree.findObject("AnnotationVisualize")
if not annotationVisualize then
  annotationVisualize = createObject("PostEffect")
  annotationVisualize:setField("shader", 0, "AnnotationVisualizeShader")
  annotationVisualize:setField("stateBlock", 0, "AL_DefaultVisualizeState")
  annotationVisualize:setField("texture", 0, "#AnnotationBuffer")
  annotationVisualize:setField("texture", 1, "shaders/common/postFx/preview_warning.png")
  annotationVisualize:setField("target", 0, "$backBuffer")
  annotationVisualize.renderPriority = 9999
  annotationVisualize:registerObject("AnnotationVisualize")
end

local function toggleLightVisualizer(objName, enable, tsVariable)
  -- log('I','lightViz', 'toggleLightVisualizer called: name = '..tostring(objName)..' enable = '..tostring(enable)..'  var = '..tostring(tsVariable))
  local vizualiser = scenetree.findObject(objName)
  if not vizualiser then
    log('E','lightViz', 'Could not find vizualiser object - '..tostring(objName))
    return
  end

  local isEnabled = vizualiser:isEnabled()
  -- log('I','lightViz', objName..'.enabled = '..tostring(isEnabled))
  if enable == nil or enable == "" then
    TorqueScriptLua.setVar(tsVariable, not isEnabled)
    vizualiser:toggle()
  elseif enable then
    vizualiser:enable()
  else
    vizualiser:disable()
  end
end
rawset(_G, "toggleLightVisualizer", toggleLightVisualizer)
rawset(_G, "toggleAnnotationVisualize",function (enable) toggleLightVisualizer("AnnotationVisualize",        enable, "$AnnotationVisualizeVar") end)
rawset(_G, "toggleDepthViz",           function (enable) toggleLightVisualizer("AL_DepthVisualize",        enable, "$AL_DepthVisualizeVar") end)
rawset(_G, "toggleNormalsViz",         function (enable) toggleLightVisualizer("AL_NormalsVisualize",      enable, "$AL_NormalsVisualizeVar") end)
rawset(_G, "toggleLightColorViz",      function (enable) toggleLightVisualizer("AL_LightColorVisualize",    enable, "$AL_LightColorVisualizeVar") end)
rawset(_G, "toggleLightSpecularViz",   function (enable) toggleLightVisualizer("AL_LightSpecularVisualize", enable, "$AL_LightSpecularVisualizeVar") end)