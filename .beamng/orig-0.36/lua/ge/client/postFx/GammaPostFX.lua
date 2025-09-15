-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local gammaPostFXCallbacks = {}

gammaPostFXCallbacks.preProcess = function()
  -- log('I', 'gammaPostFX', 'preProcess called for gammaPostFX')
  local gammaPostFX = scenetree.findObject("GammaPostFX")
  if gammaPostFX then
    local colorCorrectionRamp = TorqueScriptLua.getVar("$HDRPostFX::colorCorrectionRamp")
    local texture1 = gammaPostFX:getField("texture", "1")
    if texture1 ~= colorCorrectionRamp then
      gammaPostFX:setField("texture", 1, colorCorrectionRamp)
    end
    -- local colorCorrectionRampDefault = TorqueScriptLua.getVar("$HDRPostFX::colorCorrectionRampDefault")
    -- %combinePass.setTexture( 2, colorCorrectionRampDefault )
  end
end

gammaPostFXCallbacks.setShaderConsts = function()
  local gammaPostFX = scenetree.findObject("GammaPostFX")
  if gammaPostFX then
    local clampedGamma = clamp(settings.getValue("GraphicGamma"), 0.001, 2.2)
    gammaPostFX:setShaderConst("$OneOverGamma", 1.0 / clampedGamma);
  end
end
rawset(_G, "GammaPostFXCallbacks", gammaPostFXCallbacks)

local gammaShader = scenetree.findObject("GammaShader")
if not gammaShader then
  gammaShader = createObject("ShaderData")
  gammaShader.DXVertexShaderFile = "shaders/common/postFx/gammaP.hlsl"
  gammaShader.DXPixelShaderFile  = "shaders/common/postFx/gammaP.hlsl"
  gammaShader.pixVersion = 5.0
  gammaShader:registerObject("GammaShader")
end

local gammaStateBlock = scenetree.findObject("GammaStateBlock")
if not gammaStateBlock then
  local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
  gammaStateBlock = createObject("GFXStateBlockData")
  gammaStateBlock:inheritParentFields(pfxDefaultStateBlock)
  gammaStateBlock.samplersDefined = true
  gammaStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  gammaStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  gammaStateBlock:registerObject("GammaStateBlock")
end

local gammaPostFX = scenetree.findObject("GammaPostFX")
if not gammaPostFX then
  gammaPostFX = createObject("PostEffect")
  gammaPostFX.isEnabled = false
  gammaPostFX.allowReflectPass = false
  gammaPostFX:setField("renderTime", 0, "PFXBeforeBin")
  gammaPostFX:setField("renderBin", 0, "AfterPostFX")
  gammaPostFX.renderPriority = 9999;

  gammaPostFX:setField("shader", 0, "GammaShader")
  gammaPostFX:setField("stateBlock", 0, "GammaStateBlock")
  gammaPostFX:setField("texture", 0, "$backBuffer")

  local colorCorrectionRamp = TorqueScriptLua.getVar("$HDRPostFX::colorCorrectionRamp")
  gammaPostFX:setField("texture", 1, colorCorrectionRamp)

  local colorCorrectionRampDefault = TorqueScriptLua.getVar("$HDRPostFX::colorCorrectionRampDefault")
  gammaPostFX:setField("texture", 2, colorCorrectionRampDefault)

  gammaPostFX:registerObject("GammaPostFX")
end
