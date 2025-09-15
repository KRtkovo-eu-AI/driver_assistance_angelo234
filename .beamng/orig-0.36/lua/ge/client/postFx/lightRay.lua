-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local lightRayOccludeShader = scenetree.findObject("LightRayOccludeShader")
if not lightRayOccludeShader then
  TorqueScriptLua.setVar("$LightRayPostFX::brightScalar", 0.75)
  TorqueScriptLua.setVar("$LightRayPostFX::numSamples", 40)
  TorqueScriptLua.setVar("$LightRayPostFX::density", 0.94)
  TorqueScriptLua.setVar("$LightRayPostFX::weight", 10.0)
  TorqueScriptLua.setVar("$LightRayPostFX::decay", 1.0)
  TorqueScriptLua.setVar("$LightRayPostFX::exposure", 0.0005)
  TorqueScriptLua.setVar("$LightRayPostFX::resolutionScale", 0.20)

  lightRayOccludeShader = createObject("ShaderData")
  lightRayOccludeShader.DXVertexShaderFile = "shaders/common/postFx/lightRay/lightRayOccludeP.hlsl"
  lightRayOccludeShader.DXPixelShaderFile  = "shaders/common/postFx/lightRay/lightRayOccludeP.hlsl"
  lightRayOccludeShader.pixVersion = 5.0
  lightRayOccludeShader:registerObject("LightRayOccludeShader")
end

local lightRayShader = scenetree.findObject("LightRayShader")
if not lightRayShader then
  lightRayShader = createObject("ShaderData")
  lightRayShader.DXVertexShaderFile = "shaders/common/postFx/lightRay/lightRayP.hlsl"
  lightRayShader.DXPixelShaderFile  = "shaders/common/postFx/lightRay/lightRayP.hlsl"
  lightRayShader.pixVersion = 5.0
  lightRayShader:registerObject("LightRayShader")
end

local lightRayStateBlock = scenetree.findObject("LightRayStateBlock")
if not lightRayStateBlock then
  local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
  lightRayStateBlock = createObject("GFXStateBlockData")
  lightRayStateBlock:inheritParentFields(pfxDefaultStateBlock)
  lightRayStateBlock.samplersDefined = true
  lightRayStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  lightRayStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  lightRayStateBlock:setField("samplerStates", 2, "SamplerClampLinear")
  lightRayStateBlock:registerObject("LightRayStateBlock")
end

local lightRayPostFXCallbacks = {}

lightRayPostFXCallbacks.preProcess = function()
  local lightRayPostFX = scenetree.findObject("LightRayPostFX")
  if not lightRayPostFX then
    return
  end

  local resolutionScale = TorqueScriptLua.getVar("$LightRayPostFX::resolutionScale")
  local targetScale = string.format("%s %s", resolutionScale, resolutionScale)
  lightRayPostFX:setField("targetScale", 0, targetScale)
end

lightRayPostFXCallbacks.setShaderConsts = function()
  local lightRayPostFX = scenetree.findObject("LightRayPostFX")
  if not lightRayPostFX then
    return
  end

  lightRayPostFX:setShaderConst("$brightScalar", TorqueScriptLua.getVar("$LightRayPostFX::brightScalar"))
  local pfx = lightRayPostFX:findObjectByInternalName("final")-- scenetree.findObject("final")
  pfx = Sim.upcast(pfx)
  pfx:setShaderConst("$numSamples", TorqueScriptLua.getVar("$LightRayPostFX::numSamples"))
  pfx:setShaderConst("$density", TorqueScriptLua.getVar("$LightRayPostFX::density"))
  pfx:setShaderConst("$weight", TorqueScriptLua.getVar("$LightRayPostFX::weight"))
  pfx:setShaderConst("$decay", TorqueScriptLua.getVar("$LightRayPostFX::decay"))
  pfx:setShaderConst("$exposure", TorqueScriptLua.getVar("$LightRayPostFX::exposure"))
end
rawset(_G, "LightRayPostFXCallbacks", lightRayPostFXCallbacks)

local lightRayPostFX = scenetree.findObject("LightRayPostFX")
if not lightRayPostFX then
  lightRayPostFX = createObject("PostEffect")
  lightRayPostFX.isEnabled = false
  lightRayPostFX.allowReflectPass = false
  lightRayPostFX:setField("renderTime", 0, "PFXBeforeBin")
  lightRayPostFX:setField("renderBin", 0, "AfterPostFX")
  lightRayPostFX.renderPriority = 10;
  lightRayPostFX:setField("shader", 0, "LightRayOccludeShader")
  lightRayPostFX:setField("stateBlock", 0, "LightRayStateBlock")
  lightRayPostFX:setField("texture", 0, "$backBuffer")
  lightRayPostFX:setField("texture", 1, "#prepass[RT0]")
  lightRayPostFX:setField("texture", 2, "#prepass[Depth]")
  lightRayPostFX:setField("target", 0, "$outTex")
  lightRayPostFX:setField("targetFormat", 0, "GFXFormatR16G16B16A16F")

  lightRayPostFX:registerObject("LightRayPostFX")

  local rayShaderPFX = createObject("PostEffect")
  rayShaderPFX:setField("shader", 0, "LightRayShader")
  rayShaderPFX:setField("stateBlock", 0, "LightRayStateBlock")
  rayShaderPFX:setField("internalName", 0, "final")
  rayShaderPFX:setField("texture", 0, "$inTex")
  rayShaderPFX:setField("texture", 1, "$backBuffer")
  rayShaderPFX:setField("target", 0, "$backBuffer")
  rayShaderPFX:registerObject()
  lightRayPostFX:add(rayShaderPFX)
end