-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local ssaoPostFxCallbacks = {}

ssaoPostFxCallbacks.onEnabled = function()
  -- This tells the AL shaders to reload and sample
  -- from our #ssaoMask texture target.
  setConsoleVariable("$AL::UseSSAOMask", true)
  return true;
end

ssaoPostFxCallbacks.onDisabled = function()
  setConsoleVariable("$AL::UseSSAOMask", false)
end

local currentWasVis = nil
local currentQuality = nil
local currentTargetScale = nil
ssaoPostFxCallbacks.onAdd = function()
  -- log("I","ssaoPostFx", "ssaoPostFx called....")
  currentWasVis = "Uninitialized"
  currentQuality = "Uninitialized"
end

ssaoPostFxCallbacks.preProcess = function()
  local quality = TorqueScriptLua.getVar("$SSAOPostFx::quality")
  if quality ~= currentQuality then
    currentQuality = tostring(clamp(round(tonumber(quality)), 0, 2))
    local ssaoPostFx = scenetree.findObject("SSAOPostFx")
    if ssaoPostFx then
      ssaoPostFx:setShaderMacro( "QUALITY", currentQuality)
    end
  end
  currentTargetScale = TorqueScriptLua.getVar("$SSAOPostFx::targetScale")
end

ssaoPostFxCallbacks.setShaderConsts = function()
  local ssaoPostFx = scenetree.findObject("SSAOPostFx")
  if not ssaoPostFx then
    return
  end

  ssaoPostFx:setShaderConst("$overallStrength", TorqueScriptLua.getVar("$SSAOPostFx::overallStrength"))

  -- Abbreviate is s-small l-large.
  ssaoPostFx:setShaderConst("$sRadius", TorqueScriptLua.getVar("$SSAOPostFx::sRadius"))
  ssaoPostFx:setShaderConst("$sStrength", TorqueScriptLua.getVar("$SSAOPostFx::sStrength"))
  ssaoPostFx:setShaderConst("$sDepthMin", TorqueScriptLua.getVar("$SSAOPostFx::sDepthMin"))
  ssaoPostFx:setShaderConst("$sDepthMax", TorqueScriptLua.getVar("$SSAOPostFx::sDepthMax"))
  ssaoPostFx:setShaderConst("$sDepthPow", TorqueScriptLua.getVar("$SSAOPostFx::sDepthPow"))
  ssaoPostFx:setShaderConst("$sNormalTol", TorqueScriptLua.getVar("$SSAOPostFx::sNormalTol"))
  ssaoPostFx:setShaderConst("$sNormalPow", TorqueScriptLua.getVar("$SSAOPostFx::sNormalPow"))

  ssaoPostFx:setShaderConst("$lRadius",   TorqueScriptLua.getVar("$SSAOPostFx::lRadius"))
  ssaoPostFx:setShaderConst("$lStrength", TorqueScriptLua.getVar("$SSAOPostFx::lStrength"))
  ssaoPostFx:setShaderConst("$lDepthMin", TorqueScriptLua.getVar("$SSAOPostFx::lDepthMin"))
  ssaoPostFx:setShaderConst("$lDepthMax", TorqueScriptLua.getVar("$SSAOPostFx::lDepthMax"))
  ssaoPostFx:setShaderConst("$lDepthPow", TorqueScriptLua.getVar("$SSAOPostFx::lDepthPow"))
  ssaoPostFx:setShaderConst("$lNormalTol",TorqueScriptLua.getVar("$SSAOPostFx::lNormalTol"))
  ssaoPostFx:setShaderConst("$lNormalPow",TorqueScriptLua.getVar("$SSAOPostFx::lNormalPow"))

  -- NOTE(AK) 04/04/2022: I have to come back to thins once I know what to do as I have
  --                      no idea where the blur object comes from
  -- %blur = %this->blurY
  -- %blur.setShaderConst("$blurDepthTol", $SSAOPostFx::blurDepthTol )
  -- %blur.setShaderConst("$blurNormalTol", $SSAOPostFx::blurNormalTol )

  -- %blur = %this->blurX
  -- %blur.setShaderConst("$blurDepthTol", $SSAOPostFx::blurDepthTol )
  -- %blur.setShaderConst("$blurNormalTol", $SSAOPostFx::blurNormalTol )

  -- %blur = %this->blurY2
  -- %blur.setShaderConst("$blurDepthTol", $SSAOPostFx::blurDepthTol )
  -- %blur.setShaderConst("$blurNormalTol", $SSAOPostFx::blurNormalTol )

  -- %blur = %this->blurX2
  -- %blur.setShaderConst("$blurDepthTol", $SSAOPostFx::blurDepthTol )
  -- %blur.setShaderConst("$blurNormalTol", $SSAOPostFx::blurNormalTol )
end
rawset(_G, "SSAOPostFxCallbacks", ssaoPostFxCallbacks)

local ssaoPostFx = scenetree.findObject("SSAOPostFx")
if not ssaoPostFx then
  -- only set these when we start the game. On reloading lua, we don't want to set these values
  TorqueScriptLua.setVar("$SSAOPostFx::overallStrength", "2.0")

  -- The small radius SSAO settings.
  TorqueScriptLua.setVar("$SSAOPostFx::sRadius", "0.1")
  TorqueScriptLua.setVar("$SSAOPostFx::sStrength", "6.0")
  TorqueScriptLua.setVar("$SSAOPostFx::sDepthMin", "0.1")
  TorqueScriptLua.setVar("$SSAOPostFx::sDepthMax", "1.0")
  TorqueScriptLua.setVar("$SSAOPostFx::sDepthPow", "1.0")
  TorqueScriptLua.setVar("$SSAOPostFx::sNormalTol", "0.0")
  TorqueScriptLua.setVar("$SSAOPostFx::sNormalPow", "1.0")

  -- The large radius SSAO settings.
  TorqueScriptLua.setVar("$SSAOPostFx::lRadius", "1.0")
  TorqueScriptLua.setVar("$SSAOPostFx::lStrength", "10.0")
  TorqueScriptLua.setVar("$SSAOPostFx::lDepthMin", "0.2")
  TorqueScriptLua.setVar("$SSAOPostFx::lDepthMax", "2.0")
  TorqueScriptLua.setVar("$SSAOPostFx::lDepthPow", "0.2")
  TorqueScriptLua.setVar("$SSAOPostFx::lNormalTol", "-0.5")
  TorqueScriptLua.setVar("$SSAOPostFx::lNormalPow", "2.0")

  -- Valid values: 0, 1, 2
  TorqueScriptLua.setVar("$SSAOPostFx::quality", "0")

  --
  TorqueScriptLua.setVar("$SSAOPostFx::blurDepthTol", "0.001")

  --
  TorqueScriptLua.setVar("$SSAOPostFx::blurNormalTol", "0.95")

  --
  TorqueScriptLua.setVar("$SSAOPostFx::targetScale", "0.5 0.5")

  -----------------------------------------------------------------------------
  -- PostEffects
  -----------------------------------------------------------------------------

  ssaoPostFx = createObject("PostEffectSSAO")

  ssaoPostFx:registerObject("SSAOPostFx")
end

-- Just here for debug visualization of the
-- SSAO mask texture used during lighting.
local ssaoVizPostFx = scenetree.findObject("SSAOVizPostFx")
if not ssaoVizPostFx then
  ssaoVizPostFx = createObject("PostEffectSSAO")
  ssaoVizPostFx.allowReflectPass = false
  ssaoVizPostFx:setField("shader", 0, "PFX_PassthruShader")
  ssaoVizPostFx:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  ssaoVizPostFx:setField("texture", 0, "#ssaoMask")
  ssaoVizPostFx:setField("target", 0, "$backBuffer")
  ssaoVizPostFx:registerObject("SSAOVizPostFx")
end

local ssaoPowTableShader = scenetree.findObject("SSAOPowTableShader")
if not ssaoPowTableShader then
  ssaoPowTableShader = createObject("ShaderData")
  ssaoPowTableShader.DXVertexShaderFile    = "shaders/common/postFx/ssao/SSAO_PowerTable_V.hlsl"
  ssaoPowTableShader.DXPixelShaderFile     = "shaders/common/postFx/ssao/SSAO_PowerTable_P.hlsl"
  ssaoPowTableShader.pixVersion = 5.0
  ssaoPowTableShader:registerObject("SSAOPowTableShader")
end

local ssaoPowTablePostFx = scenetree.findObject("SSAOPowTablePostFx")
if not ssaoPowTablePostFx then
  ssaoPowTablePostFx = createObject("PostEffect")
  ssaoPowTablePostFx:setField("shader", 0, "SSAOPowTableShader")
  ssaoPowTablePostFx:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  ssaoPowTablePostFx:setField("renderTime", 0, "PFXTexGenOnDemand")
  ssaoPowTablePostFx:setField("target", 0, "#ssao_pow_table")
  ssaoPowTablePostFx:setField("targetFormat", 0, "GFXFormatR16F")
  ssaoPowTablePostFx:setField("targetSize", 0, "256 1")
  ssaoPowTablePostFx:registerObject("SSAOPowTablePostFx")
end