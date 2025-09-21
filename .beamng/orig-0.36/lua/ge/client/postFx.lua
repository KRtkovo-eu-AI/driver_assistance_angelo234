-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local dofModule = nil

local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
if not pfxDefaultStateBlock then
  pfxDefaultStateBlock = createObject("GFXStateBlockData")
  pfxDefaultStateBlock.zDefined = true
  pfxDefaultStateBlock.zEnable = false
  pfxDefaultStateBlock.zWriteEnable = false
  pfxDefaultStateBlock.samplersDefined = true
  pfxDefaultStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxDefaultStateBlock:registerObject("PFX_DefaultStateBlock")
end

local pfxDefaultBlendStateBlock = scenetree.findObject("PFX_DefaultBlitStateBlock")
if not pfxDefaultBlendStateBlock then
  pfxDefaultBlendStateBlock = createObject("GFXStateBlockData")
  pfxDefaultBlendStateBlock.zDefined = true
  pfxDefaultBlendStateBlock.zEnable = false
  pfxDefaultBlendStateBlock.zWriteEnable = false
  pfxDefaultBlendStateBlock.samplersDefined = true
  pfxDefaultBlendStateBlock.blendDefined = true;
  pfxDefaultBlendStateBlock.blendEnable = true;
  pfxDefaultBlendStateBlock:setField("blendSrc", 0, "GFXBlendSrcAlpha")
  pfxDefaultBlendStateBlock:setField("blendDest", 0, "GFXBlendInvSrcAlpha")
  pfxDefaultBlendStateBlock:setField("blendOp", 0, "GFXBlendOpAdd")
  pfxDefaultBlendStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxDefaultBlendStateBlock:registerObject("PFX_DefaultBlitStateBlock")
end

local pfxPassthruShader = scenetree.findObject("PFX_PassthruShader")
if not pfxPassthruShader then
  pfxPassthruShader = createObject("ShaderData")
  pfxPassthruShader.DXVertexShaderFile = "shaders/common/postFx/passthruP.hlsl"
  pfxPassthruShader.DXPixelShaderFile  = "shaders/common/postFx/passthruP.hlsl"
  pfxPassthruShader:setField("samplerNames", 0, "$inputTex")
  pfxPassthruShader.pixVersion = 5.0
  pfxPassthruShader:registerObject("PFX_PassthruShader")
end

local alFormatBlend = scenetree.findObject("AL_FormatBlit")
if not alFormatBlend then
  alFormatBlend = createObject("PostEffect")
  alFormatBlend.isEnabled = false
  alFormatBlend.allowReflectPass = true
  alFormatBlend.shader = "PFX_PassthruShader"
  alFormatBlend.stateBlock = scenetree.findObject("PFX_DefaultBlitStateBlock")
  alFormatBlend.texture = "$inTex"
  alFormatBlend.target = "$backbuffer"
  alFormatBlend:registerObject("AL_FormatBlit")
end

M.initPostEffects = function()
  -- First exec the scripts for the different light managers
  -- in the lighting folder.
  -- log('I', 'postFx', "initPostEffects start...");

  require("client/postFx/caustics")
  require("client/postFx/chromaticLens")
  M.loadPresetFile("lua/ge/client/postFx/presets/defaultpostfxpreset.postfx")
  dofModule = require("client/postFx/dof")
  require("client/postFx/edgeAA")
  require("client/postFx/flash")
  require('client/postFx/fog')
  require('client/postFx/fxaa')
  require('client/postFx/GammaPostFX')
  require('client/postFx/glow')
  require('client/postFx/lightRay')
  require('client/postFx/maskedScreenBlur')
  require('client/postFx/MotionBlurFx')
  require('client/postFx/smaa');
  require('client/postFx/ssao');
  require('client/postFx/turbulence');

  -- log('I', 'postFx', "... initPostEffects done");
end

M.reloadPostEffects = function()
  -- First exec the scripts for the different light managers
  -- in the lighting folder.
  -- log('I', 'postFx', "reloadPostEffects start...");

  require("client/postFx/caustics")
  require("client/postFx/chromaticLens")
  dofModule = require("client/postFx/dof")
  require("client/postFx/edgeAA")
  require("client/postFx/flash")
  require('client/postFx/fog')
  require('client/postFx/fxaa')
  require('client/postFx/GammaPostFX')
  require('client/postFx/glow')
  require('client/postFx/lightRay')
  require('client/postFx/maskedScreenBlur')
  require('client/postFx/MotionBlurFx')
  require('client/postFx/smaa');
  require('client/postFx/ssao');
  require('client/postFx/turbulence');

  -- log('I', 'postFx', "... reloadPostEffects done");
end

-- Return true if we really want the effect enabled.
-- By default this is the case.
local postEffectCallbacks = {}
postEffectCallbacks.onEnabled = function()
  -- log('I','postFx','PostEffect callback onEnable called....')
  return true
end
rawset(_G, "postEffectCallbacks", postEffectCallbacks)

local function shouldSaveCurrentValues()
  return M.backupSettings == nil
end

M.savePresetFile = function(filename)
  -- log('I','postfx','savePresetFile called: '..tostring(filename))
  filename = makeRelativePath(filename,"")

  if shouldSaveCurrentValues() then
    -- Apply the current settings to the preset
    M.settingsApplyAll()
  end

  local exports = exportToJson("$PostFXManager::Settings::*")
  -- log('I','','exported $PostFXManager::Settings::* = '..dumps(exports))
  exports.header = {version = 1}

  jsonWriteFile(filename, exports, true)

  log('I','postFx', "% - PostFX Manager - Save complete. Preset saved at : " ..filename)
end

M.loadPresetFile = function(filename)
  local presetFilename = FS:expandFilename(filename)
  if FS:fileExists(presetFilename) then
    presetFilename = makeRelativePath( presetFilename, "")
    -- log('I', 'postFx', "loadPresetFile loading: "..presetFilename)
    local preset = jsonReadFile(presetFilename)
    preset.header = nil -- remove header first before parsing to create TS flags

    for key, obj in pairs(preset) do
      local flag =  string.format("$PostFXManager::Settings::%s", key)
      if type(obj) ~= "table" then
        TorqueScriptLua.setVar(flag, obj)
      else
        for field, value in pairs(obj) do
          local fullFlag =  string.format("%s::%s", flag, field)
          TorqueScriptLua.setVar(fullFlag, value)
        end
      end
    end
    return true
  end
  return false
end

local function migrateDefaultPresetCSFile()
  local presetFilename = "settings/default.postfxpreset.cs"
  if FS:fileExists(FS:expandFilename(presetFilename)) then
    log('I', 'postfx', "PostFX Manager - migrating cs version of default preset")
    presetFilename = makeRelativePath(presetFilename, "")
    TorqueScriptLua.exec(presetFilename)
    M.settingsApplyFromPreset()
    FS:remove(presetFilename)
    return true
  end
  return false
end

M.applyDefaultPreset = function()
  -- log('I', 'postfx', "PostFX Manager - applyDefaultPreset called.....")
  TorqueScriptLua.setVar("$PostFXManager::highPreset",   "lua/ge/client/postFx/presets/defaultPostfxPreset.postfx")
  TorqueScriptLua.setVar("$PostFXManager::normalPreset", "lua/ge/client/postFx/presets/lowestPostfxPreset.postfx")
  TorqueScriptLua.setVar("$PostFXManager::lowPreset",    "lua/ge/client/postFx/presets/lowestPostfxPreset.postfx")
  TorqueScriptLua.setVar("$PostFXManager::lowestPreset", "lua/ge/client/postFx/presets/lowestPostfxPreset.postfx")

  -- Preset Migration for 1st time a user start after switch to LUA startup
  migrateDefaultPresetCSFile()

  local loaded = M.loadPresetFile("settings/postfxSettings.postfx")
  if not loaded then
    loaded = M.loadPresetFile("lua/ge/client/postFx/presets/defaultPostfxPreset.postfx")
  end
  if loaded then
    M.settingsApplyFromPreset()
  end
end

M.applySSAOPreset = function()
  TorqueScriptLua.setVar("$SSAOPostFx::Enable",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::Enable"))
  TorqueScriptLua.setVar("$SSAOPostFx::blurDepthTol",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::blurDepthTol"))
  TorqueScriptLua.setVar("$SSAOPostFx::blurNormalTol",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::blurNormalTol"))
  TorqueScriptLua.setVar("$SSAOPostFx::lDepthMax",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lDepthMax"))
  TorqueScriptLua.setVar("$SSAOPostFx::lDepthMin",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lDepthMin"))
  TorqueScriptLua.setVar("$SSAOPostFx::lDepthPow",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lDepthPow"))
  TorqueScriptLua.setVar("$SSAOPostFx::lNormalPow",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lNormalPow"))
  TorqueScriptLua.setVar("$SSAOPostFx::lNormalTol",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lNormalTol"))
  TorqueScriptLua.setVar("$SSAOPostFx::lRadius",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lRadius"))
  TorqueScriptLua.setVar("$SSAOPostFx::lStrength",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::lStrength"))
  TorqueScriptLua.setVar("$SSAOPostFx::overallStrength",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::overallStrength"))
  TorqueScriptLua.setVar("$SSAOPostFx::quality",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::quality"))
  TorqueScriptLua.setVar("$SSAOPostFx::sDepthMax",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sDepthMax"))
  TorqueScriptLua.setVar("$SSAOPostFx::sDepthMin",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sDepthMin"))
  TorqueScriptLua.setVar("$SSAOPostFx::sDepthPow",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sDepthPow"))
  TorqueScriptLua.setVar("$SSAOPostFx::sNormalPow",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sNormalPow"))
  TorqueScriptLua.setVar("$SSAOPostFx::sNormalTol",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sNormalTol"))
  TorqueScriptLua.setVar("$SSAOPostFx::sRadius",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sRadius"))
  TorqueScriptLua.setVar("$SSAOPostFx::sStrength",  TorqueScriptLua.getVar("$PostFXManager::Settings::SSAO::sStrength"))
end

M.applyHDRPreset = function()
  TorqueScriptLua.setVar("$HDRPostFX::Enable",  TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::Enable"))
  TorqueScriptLua.setVar("$HDRPostFX::adaptRate", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::adaptRate"))
  TorqueScriptLua.setVar("$HDRPostFX::blueShiftColor", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::blueShiftColor"))
  TorqueScriptLua.setVar("$HDRPostFX::brightPassThreshold", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::brightPassThreshold"))
  TorqueScriptLua.setVar("$HDRPostFX::enableBloom", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::enableBloom"))
  TorqueScriptLua.setVar("$HDRPostFX::enableBlueShift", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::enableBlueShift"))
  TorqueScriptLua.setVar("$HDRPostFX::enableToneMapping", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::enableToneMapping"))
  TorqueScriptLua.setVar("$HDRPostFX::gaussMean", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::gaussMean"))
  TorqueScriptLua.setVar("$HDRPostFX::gaussMultiplier", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::gaussMultiplier"))
  TorqueScriptLua.setVar("$HDRPostFX::gaussStdDev", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::gaussStdDev"))
  TorqueScriptLua.setVar("$HDRPostFX::keyValue", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::keyValue"))
  TorqueScriptLua.setVar("$HDRPostFX::minLuminace", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::minLuminace"))
  TorqueScriptLua.setVar("$HDRPostFX::whiteCutoff", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::whiteCutoff"))
  TorqueScriptLua.setVar("$HDRPostFX::colorCorrectionStrength", TorqueScriptLua.getVar("PostFXManager::Settings::HDR1::colorCorrectionStrength"))
  TorqueScriptLua.setVar("$HDRPostFX::colorCorrectionRamp", TorqueScriptLua.getVar("$PostFXManager::Settings::HDR1::ColorCorrectionRamp2"))
end

M.applyLightRaysPreset = function()
  TorqueScriptLua.setVar("$LightRayPostFX::Enable",  TorqueScriptLua.getVar("$PostFXManager::Settings::LightRays::Enable"))
  TorqueScriptLua.setVar("$LightRayPostFX::brightScalar", TorqueScriptLua.getVar("$PostFXManager::Settings::LightRays::brightScalar"))
end

M.applyDOFPreset = function()
  TorqueScriptLua.setVar("$DOFPostFx::Enable",  TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::Enable"))
  TorqueScriptLua.setVar("$DOFPostFx::EnableDebugMode", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::EnableDebugMode"))
  TorqueScriptLua.setVar("$DOFPostFx::BlurMin", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::BlurNear"))
  TorqueScriptLua.setVar("$DOFPostFx::BlurMax", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::BlurFar"))
  TorqueScriptLua.setVar("$DOFPostFx::FocusRangeMin", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::FocusAperture"))
  TorqueScriptLua.setVar("$DOFPostFx::FocusRangeMax", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::FocusDistance"))
  TorqueScriptLua.setVar("$DOFPostFx::BlurCurveNear", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::BlurCurveNear"))
  TorqueScriptLua.setVar("$DOFPostFx::BlurCurveFar", TorqueScriptLua.getVar("$PostFXManager::Settings::DOF::BlurCurve"))

  -- make sure we apply the correct settings to the DOF
  dofModule.updateDOFSettings()
end

M.settingsApplyFromPreset = function()
  -- log('I', 'postfx', "PostFX Manager - Applying from preset")

  -- SSAO Settings
  M.applySSAOPreset()

  -- HDR settings
  M.applyHDRPreset()

  -- Light rays settings
  M.applyLightRaysPreset()

  -- DOF settings
  M.applyDOFPreset()

  local enablePostFX = TorqueScriptLua.getBoolVar("$PostFXManager::Settings::EnablePostFX")
  TorqueScriptLua.setVar("$PostFX::Enabled",  enablePostFX)
  M.settingsSetEnabled(enablePostFX)
end

M.settingsSetEnabled = function(enablePostFX)
  TorqueScriptLua.setVar("$PostFX::Enabled", enablePostFX)
  -- if to enable the postFX, apply the ones that are enabled
  if enablePostFX then
    -- SSAO, HDR, LightRays, DOF
    local dof = scenetree.findObject("DOFPostEffect")
    local ssao = scenetree.findObject("SSAOPostFx")
    local lightRay = scenetree.findObject("LightRayPostFX")
    local hdrPostFx = scenetree.findObject("HDRPostFx")

    if ssao then
      if TorqueScriptLua.getBoolVar("$SSAOPostFx::Enable") then
        ssao:enable()
      else
        ssao:disable()
        end
    end

    if lightRay then
      if TorqueScriptLua.getBoolVar("$LightRayPostFX::Enable") then
        lightRay:enable()
      else
        lightRay:disable()
      end
    end

    if dof then
      if TorqueScriptLua.getBoolVar("$DOFPostFx::Enable") then
        dof:enable()
      else
        dof:disable()
      end
    end

    -- log('I','postfx',"PostFX Manager - PostFX enabled")
  else
    -- Disable all postFX
    if ssao then ssao:disable() end
    if hdr then hdr:disable() end
    if lightRay then lightRay:disable() end
    if dof then dof:disable() end

    -- log('I','postfx',"PostFX Manager - PostFX disabled")
  end
end

M.backupCurrentSettings = function()
  if not M.backupSettings then
    log('I','','Creating backup of Postfx settings')
    M.backupSettings = {}

    local DOF = {}
    DOF.Enable          = TorqueScriptLua.getVar('$DOFPostFx::Enable')
    DOF.EnableDebugMode = TorqueScriptLua.getVar('$DOFPostFx::EnableDebugMode')
    DOF.BlurMin         = TorqueScriptLua.getVar('$DOFPostFx::BlurMin')
    DOF.BlurMax         = TorqueScriptLua.getVar('$DOFPostFx::BlurMax')
    DOF.FocusRangeMin   = TorqueScriptLua.getVar('$DOFPostFx::FocusRangeMin')
    DOF.FocusRangeMax   = TorqueScriptLua.getVar('$DOFPostFx::FocusRangeMax')
    DOF.BlurCurveNear   = TorqueScriptLua.getVar('$DOFPostFx::BlurCurveNear')
    DOF.BlurCurveFar    = TorqueScriptLua.getVar('$DOFPostFx::BlurCurveFar')
    M.backupSettings.DOF = DOF

    local LightRay = {}
    LightRay.Enable = TorqueScriptLua.getVar('$LightRayPostFX::Enable')
    LightRay.brightScalar = TorqueScriptLua.getVar('$LightRayPostFX::brightScalar')
    M.backupSettings.LightRay = LightRay

    local HDR = {}
    HDR.Enable              = TorqueScriptLua.getVar('$HDRPostFX::Enable')
    HDR.adaptRate           = TorqueScriptLua.getVar('$HDRPostFX::adaptRate')
    HDR.blueShiftColor      = TorqueScriptLua.getVar('$HDRPostFX::blueShiftColor')
    HDR.brightPassThreshold = TorqueScriptLua.getVar('$HDRPostFX::brightPassThreshold')
    HDR.enableBloom         = TorqueScriptLua.getVar('$HDRPostFX::enableBloom')
    HDR.enableBlueShift     = TorqueScriptLua.getVar('$HDRPostFX::enableBlueShift')
    HDR.enableToneMapping   = TorqueScriptLua.getVar('$HDRPostFX::enableToneMapping')
    HDR.gaussMean           = TorqueScriptLua.getVar('$HDRPostFX::gaussMean')
    HDR.gaussMultiplier     = TorqueScriptLua.getVar('$HDRPostFX::gaussMultiplier')
    HDR.gaussStdDev         = TorqueScriptLua.getVar('$HDRPostFX::gaussStdDev')
    HDR.keyValue            = TorqueScriptLua.getVar('$HDRPostFX::keyValue')
    HDR.minLuminace             = TorqueScriptLua.getVar('$HDRPostFX::minLuminace')
    HDR.whiteCutoff             = TorqueScriptLua.getVar('$HDRPostFX::whiteCutoff')
    HDR.colorCorrectionStrength = TorqueScriptLua.getVar('$HDRPostFX::colorCorrectionStrength')
    HDR.colorCorrectionRamp     = TorqueScriptLua.getVar('$HDRPostFX::colorCorrectionRamp')
    M.backupSettings.HDR = HDR

    local SSAO = {}
    SSAO.Enable           = TorqueScriptLua.getBoolVar("$SSAOPostFx::Enable")
    SSAO.blurDepthTol     = TorqueScriptLua.getVar('$SSAOPostFx::blurDepthTol')
    SSAO.blurNormalTol    = TorqueScriptLua.getVar('$SSAOPostFx::blurNormalTol')
    SSAO.lDepthMax        = TorqueScriptLua.getVar('$SSAOPostFx::lDepthMax')
    SSAO.lDepthMin        = TorqueScriptLua.getVar('$SSAOPostFx::lDepthMin')
    SSAO.lDepthPow        = TorqueScriptLua.getVar('$SSAOPostFx::lDepthPow')
    SSAO.lNormalPow       = TorqueScriptLua.getVar('$SSAOPostFx::lNormalPow')
    SSAO.lNormalTol       = TorqueScriptLua.getVar('$SSAOPostFx::lNormalTol')
    SSAO.lRadius          = TorqueScriptLua.getVar('$SSAOPostFx::lRadius')
    SSAO.lStrength        = TorqueScriptLua.getVar('$SSAOPostFx::lStrength')
    SSAO.overallStrength  = TorqueScriptLua.getVar('$SSAOPostFx::overallStrength')
    SSAO.quality          = TorqueScriptLua.getVar('$SSAOPostFx::quality')
    SSAO.sDepthMax        = TorqueScriptLua.getVar('$SSAOPostFx::sDepthMax')
    SSAO.sDepthMin        = TorqueScriptLua.getVar('$SSAOPostFx::sDepthMin')
    SSAO.sDepthPow        = TorqueScriptLua.getVar('$SSAOPostFx::sDepthPow')
    SSAO.sNormalPow       = TorqueScriptLua.getVar('$SSAOPostFx::sNormalPow')
    SSAO.sNormalTol       = TorqueScriptLua.getVar('$SSAOPostFx::sNormalTol')
    SSAO.sRadius          = TorqueScriptLua.getVar('$SSAOPostFx::sRadius')
    SSAO.sStrength        = TorqueScriptLua.getVar('$SSAOPostFx::sStrength')
    M.backupSettings.SSAO = SSAO
  end
end

M.clearBackup = function()
  M.backupSettings = nil
end


local function settingsApplySSAO()
  TorqueScriptLua.setVar("$PostFXManager::Settings::SSAO::Enable",          TorqueScriptLua.getBoolVar("$SSAOPostFx::Enable"))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::blurDepthTol',    TorqueScriptLua.getVar('$SSAOPostFx::blurDepthTol'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::blurNormalTol',   TorqueScriptLua.getVar('$SSAOPostFx::blurNormalTol'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lDepthMax',       TorqueScriptLua.getVar('$SSAOPostFx::lDepthMax'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lDepthMin',       TorqueScriptLua.getVar('$SSAOPostFx::lDepthMin'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lDepthPow',       TorqueScriptLua.getVar('$SSAOPostFx::lDepthPow'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lNormalPow',      TorqueScriptLua.getVar('$SSAOPostFx::lNormalPow'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lNormalTol',      TorqueScriptLua.getVar('$SSAOPostFx::lNormalTol'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lRadius',         TorqueScriptLua.getVar('$SSAOPostFx::lRadius'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::lStrength',       TorqueScriptLua.getVar('$SSAOPostFx::lStrength'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::overallStrength', TorqueScriptLua.getVar('$SSAOPostFx::overallStrength'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::quality',         TorqueScriptLua.getVar('$SSAOPostFx::quality'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sDepthMax',       TorqueScriptLua.getVar('$SSAOPostFx::sDepthMax'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sDepthMin',       TorqueScriptLua.getVar('$SSAOPostFx::sDepthMin'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sDepthPow',       TorqueScriptLua.getVar('$SSAOPostFx::sDepthPow'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sNormalPow',      TorqueScriptLua.getVar('$SSAOPostFx::sNormalPow'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sNormalTol',      TorqueScriptLua.getVar('$SSAOPostFx::sNormalTol'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sRadius',         TorqueScriptLua.getVar('$SSAOPostFx::sRadius'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::SSAO::sStrength',       TorqueScriptLua.getVar('$SSAOPostFx::sStrength'))
end

local function settingsApplyHDR()
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::Enable',                  TorqueScriptLua.getVar('$HDRPostFX::Enable'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::adaptRate',               TorqueScriptLua.getVar('$HDRPostFX::adaptRate'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::blueShiftColor',          TorqueScriptLua.getVar('$HDRPostFX::blueShiftColor'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::brightPassThreshold',     TorqueScriptLua.getVar('$HDRPostFX::brightPassThreshold'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::enableBloom',             TorqueScriptLua.getVar('$HDRPostFX::enableBloom'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::enableBlueShift',         TorqueScriptLua.getVar('$HDRPostFX::enableBlueShift'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::enableToneMapping',       TorqueScriptLua.getVar('$HDRPostFX::enableToneMapping'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::gaussMean',               TorqueScriptLua.getVar('$HDRPostFX::gaussMean'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::gaussMultiplier',         TorqueScriptLua.getVar('$HDRPostFX::gaussMultiplier'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::gaussStdDev',             TorqueScriptLua.getVar('$HDRPostFX::gaussStdDev'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::keyValue',                TorqueScriptLua.getVar('$HDRPostFX::keyValue'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::minLuminace',             TorqueScriptLua.getVar('$HDRPostFX::minLuminace'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::whiteCutoff',             TorqueScriptLua.getVar('$HDRPostFX::whiteCutoff'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::colorCorrectionStrength', TorqueScriptLua.getVar('$HDRPostFX::colorCorrectionStrength'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::HDR1::ColorCorrectionRamp2',    TorqueScriptLua.getVar('$HDRPostFX::colorCorrectionRamp'))
end

local function settingsApplyLightRays()
  TorqueScriptLua.setVar('$PostFXManager::Settings::LightRays::Enable',       TorqueScriptLua.getVar('$LightRayPostFX::Enable'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::LightRays::brightScalar', TorqueScriptLua.getVar('$LightRayPostFX::brightScalar'))
end

local function settingsApplyDOF()
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::Enable',           TorqueScriptLua.getVar('$DOFPostFx::Enable'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::EnableDebugMode',  TorqueScriptLua.getVar('$DOFPostFx::EnableDebugMode'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::BlurNear',         TorqueScriptLua.getVar('$DOFPostFx::BlurMin'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::BlurFar',          TorqueScriptLua.getVar('$DOFPostFx::BlurMax'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::FocusAperture',    TorqueScriptLua.getVar('$DOFPostFx::FocusRangeMin'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::FocusDistance',    TorqueScriptLua.getVar('$DOFPostFx::FocusRangeMax'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::BlurCurveNear',    TorqueScriptLua.getVar('$DOFPostFx::BlurCurveNear'))
  TorqueScriptLua.setVar('$PostFXManager::Settings::DOF::BlurCurve',        TorqueScriptLua.getVar('$DOFPostFx::BlurCurveFar'))
end

M.settingsApplyAll = function()
  -- Apply settings which control if effects are on/off altogether.
  TorqueScriptLua.setVar("$PostFXManager::Settings::EnablePostFX", TorqueScriptLua.getBoolVar("$PostFX::Enabled"))

  -- Apply settings should save the values in the system to the
  -- the preset structure ($PostFXManager::Settings::*)

  -- SSAO Settings
  settingsApplySSAO()
  -- HDR settings
  settingsApplyHDR()
  -- Light rays settings
  settingsApplyLightRays()
  -- DOF
  settingsApplyDOF()

  -- log('I','postfx', '% - PostFX Manager - All Settings applied to $PostFXManager::Settings')
end

M.restore_cs_preset_from_json = function()
  -- log('W','postFX','restore_cs_preset_from_json callled....remember to delete this function when no longer needed')
  if M.loadPresetFile("settings/postfxSettings.postfx") then
    FS:remove("settings/postfxSettings.postfx")
  end
end
return M
