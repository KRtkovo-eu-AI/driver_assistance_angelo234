-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--
-- Default Prefs

--[[
$pref::LightManager::sgAtlasMaxDynamicLights = "16";
$pref::LightManager::sgDynamicShadowDetailSize = "0";
$pref::LightManager::sgDynamicShadowQuality = "0";
$pref::LightManager::sgLightingProfileAllowShadows = "1";
$pref::LightManager::sgLightingProfileQuality = "0";
$pref::LightManager::sgMaxBestLights = "10";
$pref::LightManager::sgMultipleDynamicShadows = "1";
$pref::LightManager::sgShowCacheStats = "0";
$pref::LightManager::sgUseBloom = "";
$pref::LightManager::sgUseDRLHighDynamicRange = "0";
$pref::LightManager::sgUseDynamicRangeLighting = "0";
$pref::LightManager::sgUseDynamicShadows = "1";
$pref::LightManager::sgUseToneMapping = "";
]]

require("/client/lighting/advanced/shaders")
require("/client/lighting/advanced/lightViz")
require("/client/lighting/advanced/shadowViz")

local advanceLighting = LightManager.findByName("Advanced Lighting")
if advanceLighting then

  local advancedLightingCallbacks = {}
  advancedLightingCallbacks.onActivate = function()
    -- log('I','ALM','Advanced Lighting onActivate called...')
    -- Don't allow the offscreen target on OSX.
    local platform = getConsoleVariable("$platform")
    if platform == "macos" then return end

    -- Enable the offscreen target so that AL will work
    -- with MSAA back buffers and for HDR rendering.
    local al_formatToken = scenetree.findObject("AL_FormatToken")
    if al_formatToken then
      al_formatToken:enable()
    end
  end
  advancedLightingCallbacks.onDeactivate = function()
    -- log('I','ALM','Advanced Lighting onDeactivate called...')
    -- Disable the offscreen render target.
    local al_formatToken = scenetree.findObject("AL_FormatToken")
    if al_formatToken then
      al_formatToken:disable()
    end
  end

  rawset(_G, "ADVLMCallbacks", advancedLightingCallbacks)
end

-- log('I','ALM', 'Advanced Lighting intialised')
-- function setAdvancedLighting()
-- {
--     setLightManager( "Advanced Lighting" );
-- }

