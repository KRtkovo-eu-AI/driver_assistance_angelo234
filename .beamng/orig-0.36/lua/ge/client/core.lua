-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local corePassthruShaderVP = scenetree.findObject("CorePassthruShaderVP")
if not corePassthruShaderVP then
  corePassthruShaderVP = createObject("ShaderData")
  corePassthruShaderVP.DXVertexShaderFile = "shaders/common/postFx/passthruV.hlsl"
  corePassthruShaderVP.DXPixelShaderFile = "shaders/common/postFx/passthruP.hlsl"
  corePassthruShaderVP:setField("samplerNames", 0, "$inputTex")
  corePassthruShaderVP.pixVersion = 5.0;
  corePassthruShaderVP:registerObject("CorePassthruShaderVP")
end

local function createCommonMaterials()
  local warningMaterial = scenetree.findObject("WarningMaterial")
  if not warningMaterial then
    warningMaterial = createObject("Material")
    warningMaterial:setField("diffuseMap", 0, "core/art/warnMat.dds")
    warningMaterial.emissive = false
    warningMaterial.translucent = false
    warningMaterial:registerObject("WarningMaterial")
  end

  local warnMatCubeMap = scenetree.findObject("WarnMatCubeMap")
  if not warnMatCubeMap then
    warnMatCubeMap = createObject("CubemapData")
    warnMatCubeMap:setField("cubeFace", 0, "core/art/warnMat.dds")
    warnMatCubeMap:setField("cubeFace", 1, "core/art/warnMat.dds")
    warnMatCubeMap:setField("cubeFace", 2, "core/art/warnMat.dds")
    warnMatCubeMap:setField("cubeFace", 3, "core/art/warnMat.dds")
    warnMatCubeMap:setField("cubeFace", 4, "core/art/warnMat.dds")
    warnMatCubeMap:setField("cubeFace", 5, "core/art/warnMat.dds")
    warnMatCubeMap:registerObject("WarnMatCubeMap")
  end

  local blankWhite = scenetree.findObject("BlankWhite")
  if not blankWhite then
    blankWhite = createObject("Material")
    blankWhite:setField("diffuseMap", 0, "core/art/white.jpg")
    blankWhite:setField("mapTo", 0, "BlankWhite")
    blankWhite:setField("materialTag0", 0, "Miscellaneous") -- not sure if this should be just materialTag, and the zero is array index?
    blankWhite:registerObject("BlankWhite")
  end

  local empty = scenetree.findObject("Empty")
  if not empty then
    empty = createObject("Material")
    empty:registerObject("Empty")
  end

  local coronaMat = scenetree.findObject("Corona_Mat")
  if not coronaMat then
    coronaMat = createObject("Material")
    coronaMat.emissive = true
    coronaMat.translucent = true
    coronaMat:setField("mapTo", 0, "corona.png")
    coronaMat:setField("diffuseMap", 0, "core/art/special/corona.png")
    coronaMat:setField("materialTag0", 0, "FX")
    coronaMat:registerObject("Corona_Mat")
  end

  local blackSkyCubemap = scenetree.findObject("BlackSkyCubemap")
  if not blackSkyCubemap then
    blackSkyCubemap = createObject("CubemapData")
    blackSkyCubemap:setField("cubeFace", 0, "core/art/skies/blank/solidsky_black.jpg")
    blackSkyCubemap:setField("cubeFace", 1, "core/art/skies/blank/solidsky_black.jpg")
    blackSkyCubemap:setField("cubeFace", 2, "core/art/skies/blank/solidsky_black.jpg")
    blackSkyCubemap:setField("cubeFace", 3, "core/art/skies/blank/solidsky_black.jpg")
    blackSkyCubemap:setField("cubeFace", 4, "core/art/skies/blank/solidsky_black.jpg")
    blackSkyCubemap:setField("cubeFace", 5, "core/art/skies/blank/solidsky_black.jpg")
    blackSkyCubemap:registerObject("BlackSkyCubemap")
  end

  local blackSkyMat = scenetree.findObject("BlackSkyMat")
  if not blackSkyMat then
    blackSkyMat = createObject("Material")
    blackSkyMat:setField("cubemap", 0, "BlackSkyCubemap")
    blackSkyMat:setField("materialTag0", 0, "Skies")
    blackSkyMat:registerObject("BlackSkyMat")
  end

  local blueSkyCubemap = scenetree.findObject("BlueSkyCubemap")
  if not blueSkyCubemap then
    blueSkyCubemap = createObject("CubemapData")
    blueSkyCubemap:setField("cubeFace", 0, "core/art/skies/blank/solidsky_blue.jpg")
    blueSkyCubemap:setField("cubeFace", 1, "core/art/skies/blank/solidsky_blue.jpg")
    blueSkyCubemap:setField("cubeFace", 2, "core/art/skies/blank/solidsky_blue.jpg")
    blueSkyCubemap:setField("cubeFace", 3, "core/art/skies/blank/solidsky_blue.jpg")
    blueSkyCubemap:setField("cubeFace", 4, "core/art/skies/blank/solidsky_blue.jpg")
    blueSkyCubemap:setField("cubeFace", 5, "core/art/skies/blank/solidsky_blue.jpg")
    blueSkyCubemap:registerObject("BlueSkyCubemap")
  end

  local blueSkyMat = scenetree.findObject("BlueSkyMat")
  if not blueSkyMat then
    blueSkyMat = createObject("Material")
    blueSkyMat:setField("cubemap", 0, "BlueSkyCubemap")
    blueSkyMat:setField("materialTag0", 0, "Skies")
    blueSkyMat:registerObject("BlueSkyMat")
  end

  local greySkyCubemap = scenetree.findObject("GreySkyCubemap")
  if not greySkyCubemap then
    greySkyCubemap = createObject("CubemapData")
    greySkyCubemap:setField("cubeFace", 0, "core/art/skies/blank/solidsky_grey.jpg")
    greySkyCubemap:setField("cubeFace", 1, "core/art/skies/blank/solidsky_grey.jpg")
    greySkyCubemap:setField("cubeFace", 2, "core/art/skies/blank/solidsky_grey.jpg")
    greySkyCubemap:setField("cubeFace", 3, "core/art/skies/blank/solidsky_grey.jpg")
    greySkyCubemap:setField("cubeFace", 4, "core/art/skies/blank/solidsky_grey.jpg")
    greySkyCubemap:setField("cubeFace", 5, "core/art/skies/blank/solidsky_grey.jpg")
    greySkyCubemap:registerObject("GreySkyCubemap")
  end

  local greySkyMat = scenetree.findObject("GreySkyMat")
  if not greySkyMat then
    greySkyMat = createObject("Material")
    greySkyMat:setField("cubemap", 0, "GreySkyCubemap")
    greySkyMat:setField("materialTag0", 0, "Skies")
    greySkyMat:registerObject("GreySkyMat")
  end

  local octahedronMat = scenetree.findObject("OctahedronMat")
  if not octahedronMat then
    octahedronMat = createObject("Material")
    octahedronMat:setField("mapTo", 0, "green")
    octahedronMat:setField("diffuseMap", 0, "/core/art/shapes/camera.png")
    octahedronMat:setField("translucent", 0, "1")
    octahedronMat:setField("translucentBlendOp", 0, "LerpAlpha")
    octahedronMat:setField("emissive", 0, "0")
    octahedronMat:setField("castShadows", 0, "0")
    octahedronMat:setField("colorMultiply", 0, "0 1 0 1")
    octahedronMat:registerObject("OctahedronMat")
  end

  local simpleConeMat = scenetree.findObject("SimpleConeMat")
  if not simpleConeMat then
    simpleConeMat = createObject("Material")
    simpleConeMat:setField("mapTo", 0, "blue")
    simpleConeMat:setField("diffuseMap", 0, "/core/art/shapes/blue.jpg")
    simpleConeMat:setField("translucent", 0, "0")
    simpleConeMat:setField("emissive", 0, "1")
    simpleConeMat:setField("castShadows", 0, "0")
    simpleConeMat:registerObject("SimpleConeMat")
  end

  local cameraMat = scenetree.findObject("CameraMat")
  if not cameraMat then
    cameraMat = createObject("Material")
    cameraMat:setField("mapTo", 0, "CameraMat")
    cameraMat:setField("diffuseMap", 0, "/core/art/shapes/blue.jpg")
    cameraMat:setField("diffuseColor", 0, "1 1 1 0.5")
    cameraMat:setField("specular", 0, "1 1 1 1")
    cameraMat:setField("specularPower", 0, "211")
    cameraMat:setField("pixelSpecular", 0, "1")
    cameraMat:setField("emissive", 0, "0")
    cameraMat.doubleSided = true
    cameraMat.translucent = true
    cameraMat:setField("translucentBlendOp", 0, "LerpAlpha")
    cameraMat.castShadows = false
    cameraMat:setField("materialTag0", 0, "Miscellaneous")
    cameraMat:registerObject("CameraMat")
  end

  local noshapeNoShape = scenetree.findObject("noshape_NoShape")
  if not noshapeNoShape then
    noshapeNoShape = createObject("Material")
    noshapeNoShape:setField("mapTo", 0, "NoShape")
    noshapeNoShape:setField("diffuseMap", 0, "")
    noshapeNoShape:setField("diffuseColor", 0, "0.8 0.003067 0 .8")
    noshapeNoShape:setField("emissive", 0, "0")
    noshapeNoShape.doubleSided = false
    noshapeNoShape.translucent = true
    noshapeNoShape:setField("translucentBlendOp", 0, "LerpAlpha")
    noshapeNoShape.castShadows = false
    noshapeNoShape:registerObject("noshape_NoShape")
  end

  local noshapetextLambert1 = scenetree.findObject("noshapetext_lambert1")
  if not noshapetextLambert1 then
    noshapetextLambert1 = createObject("Material")
    noshapetextLambert1:setField("mapTo", 0, "lambert1")
    noshapetextLambert1:setField("diffuseMap", 0, "")
    noshapetextLambert1:setField("diffuseColor", 0, "0.4 0.4 0.4 1")
    noshapetextLambert1:setField("specular", 0, "1 1 1 1")
    noshapetextLambert1:setField("specularPower", 0, "8")
    noshapetextLambert1:setField("pixelSpecular", 0, "0")
    noshapetextLambert1:setField("emissive", 0, "1")
    noshapetextLambert1.doubleSided = false
    noshapetextLambert1.translucent = false
    noshapetextLambert1:setField("translucentBlendOp", 0, "None")
    noshapetextLambert1:registerObject("noshapetext_lambert1")
  end

  local noshapetextNoshapeMat = scenetree.findObject("noshapetext_noshape_mat")
  if not noshapetextNoshapeMat then
    noshapetextNoshapeMat = createObject("Material")
    noshapetextNoshapeMat:setField("mapTo", 0, "noshape_mat")
    noshapetextNoshapeMat:setField("diffuseMap", 0, "")
    noshapetextNoshapeMat:setField("diffuseColor", 0, "0.4 0.3504 0.363784 0.33058")
    noshapetextNoshapeMat:setField("specular", 0, "1 1 1 1")
    noshapetextNoshapeMat:setField("specularPower", 0, "8")
    noshapetextNoshapeMat:setField("pixelSpecular", 0, "0")
    noshapetextNoshapeMat:setField("emissive", 0, "1")
    noshapetextNoshapeMat.doubleSided = false
    noshapetextNoshapeMat.translucent = true
    noshapetextNoshapeMat:setField("translucentBlendOp", 0, "None")
    noshapetextNoshapeMat:registerObject("noshapetext_noshape_mat")
  end

  local noshapetextNoshapeMat = scenetree.findObject("portal5_portal_top")
  if not noshapetextNoshapeMat then
    noshapetextNoshapeMat = createObject("Material")
    noshapetextNoshapeMat:setField("mapTo", 0, "portal_top")
    noshapetextNoshapeMat:setField("diffuseMap", 0, "/core/art/shapes/top.png")
    noshapetextNoshapeMat:setField("normalMap", 0, "/core/art/shapes/top-normal.png")
    noshapetextNoshapeMat:setField("diffuseColor", 0, "0.4 0.4 0.4 1")
    noshapetextNoshapeMat:setField("specular", 0, "0.5 0.5 0.5 1")
    noshapetextNoshapeMat:setField("specularPower", 0, "2")
    noshapetextNoshapeMat:setField("pixelSpecular", 0, "0")
    noshapetextNoshapeMat:setField("emissive", 0, "1")
    noshapetextNoshapeMat.doubleSided = false
    noshapetextNoshapeMat.translucent = false
    noshapetextNoshapeMat:setField("translucentBlendOp", 0, "None")
    noshapetextNoshapeMat:registerObject("portal5_portal_top")
  end

  local portal5PortalLightray = scenetree.findObject("portal5_portal_lightray")
  if not portal5PortalLightray then
    portal5PortalLightray = createObject("Material")
    portal5PortalLightray:setField("mapTo", 0, "portal_lightray")
    portal5PortalLightray:setField("diffuseMap", 0, "/core/art/shapes/lightray.png")
    portal5PortalLightray:setField("diffuseColor", 0, "0.4 0.4 0.4 0.64462")
    portal5PortalLightray:setField("specular", 0, "0.5 0.5 0.5 1")
    portal5PortalLightray:setField("specularPower", 0, "2")
    portal5PortalLightray:setField("pixelSpecular", 0, "0")
    portal5PortalLightray:setField("emissive", 0, "1")
    portal5PortalLightray.doubleSided = true
    portal5PortalLightray.translucent = true
    portal5PortalLightray:setField("translucentBlendOp", 0, "AddAlpha")
    portal5PortalLightray.castShadows = false
    portal5PortalLightray:registerObject("portal5_portal_lightray")
  end

  local spawnArrow = scenetree.findObject("spawn_arrow")
  if not spawnArrow then
    spawnArrow = createObject("Material")
    spawnArrow:setField("mapTo", 0, "spawn_arrow")
    spawnArrow:setField("diffuseColor", 0, "1 0.455 0 0.85")
    spawnArrow:setField("colorMap", 0, "/core/art/white.jpg")
    spawnArrow:setField("emissive", 0, "1")
    spawnArrow.translucent = true
    spawnArrow:registerObject("spawn_arrow")
  end

  local grid512BlackMat = scenetree.findObject("Grid512_Black_Mat")
  if not grid512BlackMat then
    grid512BlackMat = createObject("Material")
    grid512BlackMat:setField("mapTo", 0, "Grid512_Black_Mat")
    grid512BlackMat:setField("diffuseMap", 0, "512_black")
    grid512BlackMat:setField("materialTag0", 0, "TestMaterial")
    grid512BlackMat:registerObject("Grid512_Black_Mat")
  end

  local grid512BlueMat = scenetree.findObject("Grid512_Blue_Mat")
  if not grid512BlueMat then
    grid512BlueMat = createObject("Material")
    grid512BlueMat:setField("mapTo", 0, "Grid512_Blue_Mat")
    grid512BlueMat:setField("diffuseMap", 0, "512_blue")
    grid512BlueMat:setField("materialTag0", 0, "TestMaterial")
    grid512BlueMat:registerObject("Grid512_Blue_Mat")
  end

  local grid512ForestGreenMat = scenetree.findObject("Grid512_ForestGreen_Mat")
  if not grid512ForestGreenMat then
    grid512ForestGreenMat = createObject("Material")
    grid512ForestGreenMat:setField("mapTo", 0, "Grid512_ForestGreen_Mat")
    grid512ForestGreenMat:setField("diffuseMap", 0, "512_forestgreen")
    grid512ForestGreenMat:setField("materialTag0", 0, "TestMaterial")
    grid512ForestGreenMat:registerObject("Grid512_ForestGreen_Mat")
  end

  local grid512ForestGreenMat = scenetree.findObject("Grid512_ForestGreenLines_Mat")
  if not grid512ForestGreenMat then
    grid512ForestGreenMat = createObject("Material")
    grid512ForestGreenMat:setField("mapTo", 0, "Grid512_ForestGreenLines_Mat")
    grid512ForestGreenMat:setField("diffuseMap", 0, "512_forestgreen_lines")
    grid512ForestGreenMat:setField("materialTag0", 0, "TestMaterial")
    grid512ForestGreenMat:registerObject("Grid512_ForestGreenLines_Mat")
  end

  local grid512GreenMat = scenetree.findObject("Grid512_Green_Mat")
  if not grid512GreenMat then
    grid512GreenMat = createObject("Material")
    grid512GreenMat:setField("mapTo", 0, "Grid512_Green_Mat")
    grid512GreenMat:setField("diffuseMap", 0, "512_green")
    grid512GreenMat:setField("materialTag0", 0, "TestMaterial")
    grid512GreenMat:registerObject("Grid512_Green_Mat")
  end

  local grid512GreyMat = scenetree.findObject("Grid512_Grey_Mat")
  if not grid512GreyMat then
    grid512GreyMat = createObject("Material")
    grid512GreyMat:setField("mapTo", 0, "Grid512_Grey_Mat")
    grid512GreyMat:setField("diffuseMap", 0, "512_grey")
    grid512GreyMat:setField("materialTag0", 0, "TestMaterial")
    grid512GreyMat:registerObject("Grid512_Grey_Mat")
  end

  local grid512GreyMat = scenetree.findObject("Grid512_GreyBase_Mat")
  if not grid512GreyMat then
    grid512GreyMat = createObject("Material")
    grid512GreyMat:setField("mapTo", 0, "Grid512_GreyBase_Mat")
    grid512GreyMat:setField("diffuseMap", 0, "512_grey")
    grid512GreyMat:setField("materialTag0", 0, "TestMaterial")
    grid512GreyMat:registerObject("Grid512_GreyBase_Mat")
  end

  local grid512OrangeMat = scenetree.findObject("Grid512_Orange_Mat")
  if not grid512OrangeMat then
    grid512OrangeMat = createObject("Material")
    grid512OrangeMat:setField("mapTo", 0, "Grid512_Orange_Mat")
    grid512OrangeMat:setField("diffuseMap", 0, "512_orange")
    grid512OrangeMat:setField("materialTag0", 0, "TestMaterial")
    grid512OrangeMat:registerObject("Grid512_Orange_Mat")
  end

  local grid512OrangeLinesMat = scenetree.findObject("Grid512_OrangeLines_Mat")
  if not grid512OrangeLinesMat then
    grid512OrangeLinesMat = createObject("Material")
    grid512OrangeLinesMat:setField("mapTo", 0, "Grid512_OrangeLines_Mat")
    grid512OrangeLinesMat:setField("diffuseMap", 0, "512_orange_lines")
    grid512OrangeLinesMat:setField("materialTag0", 0, "TestMaterial")
    grid512OrangeLinesMat:registerObject("Grid512_OrangeLines_Mat")
  end

  local grid512RedMat = scenetree.findObject("Grid512_Red_Mat")
  if not grid512RedMat then
    grid512RedMat = createObject("Material")
    grid512RedMat:setField("mapTo", 0, "Grid512_Red_Mat")
    grid512RedMat:setField("diffuseMap", 0, "512_red")
    grid512RedMat:setField("materialTag0", 0, "TestMaterial")
    grid512RedMat:registerObject("Grid512_Red_Mat")
  end
end

local function createTrackbuilderMaterials()
  -- log('I',logTag,'creating trackbuilder materials....')
  local track_editor_grid = scenetree.findObject("track_editor_grid")
  if not track_editor_grid then
    track_editor_grid = createObject("Material")
    track_editor_grid.mapTo = "track_editor_grid"
    track_editor_grid.materialTag0 = "TestMaterial"
    track_editor_grid:setField("useAnisotropic", 0, "1")
    track_editor_grid:setField("doubleSided", 0, "0")
    track_editor_grid:setField("scrollSpeed", 0, "1")
    track_editor_grid:setField("diffuseColor", 0, "1 1 1 1")
    track_editor_grid:setField("colorMap", 0, "core/art/trackBuilder/track_editor_grid.png")
    track_editor_grid:setField("emissive", 0, "1")
    track_editor_grid:registerObject("track_editor_grid")
  end

  local track_editor_A_center = scenetree.findObject("track_editor_A_center")
  if not track_editor_A_center then
    track_editor_A_center = createObject("Material")
    track_editor_A_center.mapTo = "track_editor_A_center"
    track_editor_A_center:setField("diffuseColor", 0, "0.803922 0.803922 0.803922 1")
    track_editor_A_center:setField("specularPower", 0, "50")
    track_editor_A_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_A_center:setField("specular", 0, "1 1 1 1")
    track_editor_A_center:setField("useAnisotropic", 0, "1")
    track_editor_A_center:setField("doubleSided", 0, "0")
    track_editor_A_center:setField("translucentBlendOp", 0, "None")
    track_editor_A_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_A_center:setField("materialTag0", 0, "beamng")
    track_editor_A_center:setField("diffuseColor", 1, "0 0 0 0.537254989")
    track_editor_A_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_A_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_A_center:setField("useAnisotropic", 1, "1")
    track_editor_A_center:setField("groundType", 0, "ASPHALT")
    track_editor_A_center:registerObject("track_editor_A_center")
  end

  local track_editor_A_border = scenetree.findObject("track_editor_A_border")
  if not track_editor_A_border then
    track_editor_A_border = createObject("Material")
    track_editor_A_border.mapTo = "track_editor_A_border"
    track_editor_A_border:setField("diffuseColor", 0, "0.803921998 0.803921998 0.803921998 1")
    track_editor_A_border:setField("diffuseColor", 1, "0.882353008 0.313726008 0 1")
    track_editor_A_border:setField("specularPower", 0, "50")
    track_editor_A_border:setField("specular", 0, "1 1 1 1")
    track_editor_A_border:setField("useAnisotropic", 0, "1")
    track_editor_A_border:setField("useAnisotropic", 1, "1")
    track_editor_A_border:setField("doubleSided", 0, "0")
    track_editor_A_border:setField("translucentBlendOp", 0, "None")
    track_editor_A_border:setField("materialTag0", 0, "beamng")
    track_editor_A_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_A_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_A_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_decal.dds")
    track_editor_A_border:setField("glow", 1, "0")
    track_editor_A_border:setField("emissive", 1, "0")
    track_editor_A_border:setField("groundType", 0, "ASPHALT")
    track_editor_A_border:registerObject("track_editor_A_border")
  end

  local track_editor_B_center = scenetree.findObject("track_editor_B_center")
  if not track_editor_B_center then
    track_editor_B_center = createObject("Material")
    track_editor_B_center.mapTo = "track_editor_B_center"
    track_editor_B_center:setField("diffuseColor", 0, "0.803922 0.803922 0.803922 1")
    track_editor_B_center:setField("diffuseColor", 1, "0 0 0 0.535911977")
    track_editor_B_center:setField("specularPower", 0, "50")
    track_editor_B_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_B_center:setField("specular", 0, "1 1 1 1")
    track_editor_B_center:setField("useAnisotropic", 0, "1")
    track_editor_B_center:setField("useAnisotropic", 1, "1")
    track_editor_B_center:setField("doubleSided", 0, "0")
    track_editor_B_center:setField("translucentBlendOp", 0, "None")
    track_editor_B_center:setField("materialTag0", 0, "beamng")
    track_editor_B_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_B_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_B_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_B_center:setField("groundType", 0, "ASPHALT")
    track_editor_B_center:registerObject("track_editor_B_center")
  end

  local track_editor_B_border = scenetree.findObject("track_editor_B_border")
  if not track_editor_B_border then
    track_editor_B_border = createObject("Material")
    track_editor_B_border.mapTo = "track_editor_B_border"
    track_editor_B_border:setField("diffuseColor", 0, "0.803921998 0.803921998 0.803921998 1")
    track_editor_B_border:setField("diffuseColor", 1, "0.339011997 0.834254026 0.207411006 1")
    track_editor_B_border:setField("specularPower", 0, "50")
    track_editor_B_border:setField("specular", 0, "1 1 1 1")
    track_editor_B_border:setField("useAnisotropic", 0, "1")
    track_editor_B_border:setField("useAnisotropic", 1, "1")
    track_editor_B_border:setField("doubleSided", 0, "0")
    track_editor_B_border:setField("translucentBlendOp", 0, "None")
    track_editor_B_border:setField("materialTag0", 0, "beamng")
    track_editor_B_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_B_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_B_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_decal.dds")
    track_editor_B_border:setField("glow", 1, "0")
    track_editor_B_border:setField("emissive", 1, "0")
    track_editor_B_border:setField("groundType", 0, "ASPHALT")
    track_editor_B_border:registerObject("track_editor_B_border")
  end

  local track_editor_C_center = scenetree.findObject("track_editor_C_center")
  if not track_editor_C_center then
    track_editor_C_center = createObject("Material")
    track_editor_C_center.mapTo = "track_editor_C_center"
    track_editor_C_center:setField("diffuseColor", 0, "0.803922 0.803922 0.803922 1")
    track_editor_C_center:setField("diffuseColor", 1, "0 0 0 0.535911977")
    track_editor_C_center:setField("specularPower", 0, "50")
    track_editor_C_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_C_center:setField("specular", 0, "1 1 1 1")
    track_editor_C_center:setField("useAnisotropic", 0, "1")
    track_editor_C_center:setField("useAnisotropic", 1, "1")
    track_editor_C_center:setField("doubleSided", 0, "0")
    track_editor_C_center:setField("translucentBlendOp", 0, "None")
    track_editor_C_center:setField("materialTag0", 0, "beamng")
    track_editor_C_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_C_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_C_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_C_center:setField("groundType", 0, "ASPHALT")
    track_editor_C_center:registerObject("track_editor_C_center")
  end

  local track_editor_C_border = scenetree.findObject("track_editor_C_border")
  if not track_editor_C_border then
    track_editor_C_border = createObject("Material")
    track_editor_C_border.mapTo = "track_editor_C_border"
    track_editor_C_border:setField("diffuseColor", 0, "0.803921998 0.803921998 0.803921998 1")
    track_editor_C_border:setField("diffuseColor", 1, "0.121546999 0.43215999 1 1")
    track_editor_C_border:setField("specularPower", 0, "50")
    track_editor_C_border:setField("specular", 0, "1 1 1 1")
    track_editor_C_border:setField("useAnisotropic", 0, "1")
    track_editor_C_border:setField("useAnisotropic", 1, "1")
    track_editor_C_border:setField("doubleSided", 0, "0")
    track_editor_C_border:setField("translucentBlendOp", 0, "None")
    track_editor_C_border:setField("materialTag0", 0, "beamng")
    track_editor_C_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_C_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_C_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_decal.dds")
    track_editor_C_border:setField("glow", 1, "0")
    track_editor_C_border:setField("emissive", 1, "0")
    track_editor_C_border:setField("groundType", 0, "ASPHALT")
    track_editor_C_border:registerObject("track_editor_C_border")
  end

  local track_editor_D_center = scenetree.findObject("track_editor_D_center")
  if not track_editor_D_center then
    track_editor_D_center = createObject("Material")
    track_editor_D_center.mapTo = "track_editor_D_center"
    track_editor_D_center:setField("diffuseColor", 0, "0.803922 0.803922 0.803922 1")
    track_editor_D_center:setField("diffuseColor", 1, "0 0 0 0.535911977")
    track_editor_D_center:setField("specularPower", 0, "50")
    track_editor_D_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_D_center:setField("specular", 0, "1 1 1 1")
    track_editor_D_center:setField("useAnisotropic", 0, "1")
    track_editor_D_center:setField("useAnisotropic", 1, "1")
    track_editor_D_center:setField("doubleSided", 0, "0")
    track_editor_D_center:setField("translucentBlendOp", 0, "None")
    track_editor_D_center:setField("materialTag0", 0, "beamng")
    track_editor_D_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_D_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_D_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_D_center:setField("groundType", 0, "ASPHALT")
    track_editor_D_center:registerObject("track_editor_D_center")
  end

  local track_editor_D_border = scenetree.findObject("track_editor_D_border")
  if not track_editor_D_border then
    track_editor_D_border = createObject("Material")
    track_editor_D_border.mapTo = "track_editor_D_border"
    track_editor_D_border:setField("diffuseColor", 0, "0.803921998 0.803921998 0.803921998 1")
    track_editor_D_border:setField("diffuseColor", 1, "0.928176999 0.199689999 0.102560997 1")
    track_editor_D_border:setField("specularPower", 0, "50")
    track_editor_D_border:setField("specular", 0, "1 1 1 1")
    track_editor_D_border:setField("useAnisotropic", 0, "1")
    track_editor_D_border:setField("useAnisotropic", 1, "1")
    track_editor_D_border:setField("doubleSided", 0, "0")
    track_editor_D_border:setField("translucentBlendOp", 0, "None")
    track_editor_D_border:setField("materialTag0", 0, "beamng")
    track_editor_D_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_D_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_D_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_decal.dds")
    track_editor_D_border:setField("glow", 1, "0")
    track_editor_D_border:setField("emissive", 1, "0")
    track_editor_D_border:setField("groundType", 0, "ASPHALT")
    track_editor_D_border:registerObject("track_editor_D_border")
  end

  local track_editor_E_center = scenetree.findObject("track_editor_E_center")
  if not track_editor_E_center then
    track_editor_E_center = createObject("Material")
    track_editor_E_center.mapTo = "track_editor_E_center"
    track_editor_E_center:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_E_center:setField("diffuseColor", 1, "0.845304012 0.417324007 0.0467020012 1")
    track_editor_E_center:setField("specularPower", 0, "50")
    track_editor_E_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_E_center:setField("specular", 0, "1 1 1 1")
    track_editor_E_center:setField("useAnisotropic", 0, "1")
    track_editor_E_center:setField("useAnisotropic", 1, "1")
    track_editor_E_center:setField("doubleSided", 0, "0")
    track_editor_E_center:setField("translucentBlendOp", 0, "None")
    track_editor_E_center:setField("materialTag0", 0, "beamng")
    track_editor_E_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_E_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_E_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_E_center:setField("glow", 1, "1")
    track_editor_E_center:setField("emissive", 1, "1")
    track_editor_E_center:setField("groundType", 0, "ASPHALT")
    track_editor_E_center:registerObject("track_editor_E_center")
  end

  local track_editor_E_border = scenetree.findObject("track_editor_E_border")
  if not track_editor_E_border then
    track_editor_E_border = createObject("Material")
    track_editor_E_border.mapTo = "track_editor_E_border"
    track_editor_E_border:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_E_border:setField("diffuseColor", 1, "0.883978009 0.311995 0 1")
    track_editor_E_border:setField("specularPower", 0, "50")
    track_editor_E_border:setField("specular", 0, "1 1 1 1")
    track_editor_E_border:setField("useAnisotropic", 0, "1")
    track_editor_E_border:setField("useAnisotropic", 1, "1")
    track_editor_E_border:setField("doubleSided", 0, "0")
    track_editor_E_border:setField("translucentBlendOp", 0, "None")
    track_editor_E_border:setField("materialTag0", 0, "beamng")
    track_editor_E_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_E_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_E_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_raw_decal.dds")
    track_editor_E_border:setField("glow", 1, "1")
    track_editor_E_border:setField("emissive", 1, "1")
    track_editor_E_border:setField("groundType", 0, "ASPHALT")

    track_editor_E_border:registerObject("track_editor_E_border")
  end

  local track_editor_F_center = scenetree.findObject("track_editor_F_center")
  if not track_editor_F_center then
    track_editor_F_center = createObject("Material")
    track_editor_F_center.mapTo = "track_editor_F_center"
    track_editor_F_center:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_F_center:setField("diffuseColor", 1, "0.0537680015 0.823203981 0.0181920007 0.535911977")
    track_editor_F_center:setField("specularPower", 0, "50")
    track_editor_F_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_F_center:setField("specular", 0, "1 1 1 1")
    track_editor_F_center:setField("useAnisotropic", 0, "1")
    track_editor_F_center:setField("useAnisotropic", 1, "1")
    track_editor_F_center:setField("doubleSided", 0, "0")
    track_editor_F_center:setField("translucentBlendOp", 0, "None")
    track_editor_F_center:setField("materialTag0", 0, "beamng")
    track_editor_F_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_F_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_F_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_F_center:setField("glow", 1, "1")
    track_editor_F_center:setField("emissive", 1, "1")
    track_editor_F_center:setField("groundType", 0, "ASPHALT")
    track_editor_F_center:registerObject("track_editor_F_center")
  end

  local track_editor_F_border = scenetree.findObject("track_editor_F_border")
  if not track_editor_F_border then
    track_editor_F_border = createObject("Material")
    track_editor_F_border.mapTo = "track_editor_F_border"
    track_editor_F_border:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_F_border:setField("diffuseColor", 1, "0.480693012 1 0.121546999 1")
    track_editor_F_border:setField("specularPower", 0, "50")
    track_editor_F_border:setField("specular", 0, "1 1 1 1")
    track_editor_F_border:setField("useAnisotropic", 0, "1")
    track_editor_F_border:setField("useAnisotropic", 1, "1")
    track_editor_F_border:setField("doubleSided", 0, "0")
    track_editor_F_border:setField("translucentBlendOp", 0, "None")
    track_editor_F_border:setField("materialTag0", 0, "beamng")
    track_editor_F_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_F_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_F_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_raw_decal.dds")
    track_editor_F_border:setField("glow", 1, "1")
    track_editor_F_border:setField("emissive", 1, "1")
    track_editor_F_border:setField("groundType", 0, "ASPHALT")
    track_editor_F_border:registerObject("track_editor_F_border")
  end

  local track_editor_G_center = scenetree.findObject("track_editor_G_center")
  if not track_editor_G_center then
    track_editor_G_center = createObject("Material")
    track_editor_G_center.mapTo = "track_editor_G_center"
    track_editor_G_center:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_G_center:setField("diffuseColor", 1, "0 0.702385008 1 0.535911977")
    track_editor_G_center:setField("specularPower", 0, "50")
    track_editor_G_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_G_center:setField("specular", 0, "1 1 1 1")
    track_editor_G_center:setField("useAnisotropic", 0, "1")
    track_editor_G_center:setField("useAnisotropic", 1, "1")
    track_editor_G_center:setField("doubleSided", 0, "0")
    track_editor_G_center:setField("translucentBlendOp", 0, "None")
    track_editor_G_center:setField("materialTag0", 0, "beamng")
    track_editor_G_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_G_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_G_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_G_center:setField("glow", 1, "1")
    track_editor_G_center:setField("emissive", 1, "1")
    track_editor_G_center:setField("groundType", 0, "ASPHALT")
    track_editor_G_center:registerObject("track_editor_G_center")
  end

  local track_editor_G_border = scenetree.findObject("track_editor_G_border")
  if not track_editor_G_border then
    track_editor_G_border = createObject("Material")
    track_editor_G_border.mapTo = "track_editor_G_border"
    track_editor_G_border:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_G_border:setField("diffuseColor", 1, "0 0.30084601 0.850829005 1")
    track_editor_G_border:setField("specularPower", 0, "50")
    track_editor_G_border:setField("specular", 0, "1 1 1 1")
    track_editor_G_border:setField("useAnisotropic", 0, "1")
    track_editor_G_border:setField("useAnisotropic", 1, "1")
    track_editor_G_border:setField("doubleSided", 0, "0")
    track_editor_G_border:setField("translucentBlendOp", 0, "None")
    track_editor_G_border:setField("materialTag0", 0, "beamng")
    track_editor_G_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_G_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_G_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_raw_decal.dds")
    track_editor_G_border:setField("glow", 1, "1")
    track_editor_G_border:setField("emissive", 1, "1")
    track_editor_G_border:setField("groundType", 0, "ASPHALT")
    track_editor_G_border:registerObject("track_editor_G_border")
  end

  local track_editor_H_center = scenetree.findObject("track_editor_H_center")
  if not track_editor_H_center then
    track_editor_H_center = createObject("Material")
    track_editor_H_center.mapTo = "track_editor_H_center"
    track_editor_H_center:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_H_center:setField("diffuseColor", 1, "1 0 0 0.701656997")
    track_editor_H_center:setField("specularPower", 0, "50")
    track_editor_H_center:setField("specularMap", 0, "core/art/trackBuilder/track_editor_base_s.dds")
    track_editor_H_center:setField("specular", 0, "1 1 1 1")
    track_editor_H_center:setField("useAnisotropic", 0, "1")
    track_editor_H_center:setField("useAnisotropic", 1, "1")
    track_editor_H_center:setField("doubleSided", 0, "0")
    track_editor_H_center:setField("translucentBlendOp", 0, "None")
    track_editor_H_center:setField("materialTag0", 0, "beamng")
    track_editor_H_center:setField("materialTag1", 0, "RoadAndPath")
    track_editor_H_center:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_H_center:setField("colorMap", 1, "core/art/trackBuilder/track_editor_line_center_decal.dds")
    track_editor_H_center:setField("glow", 1, "1")
    track_editor_H_center:setField("emissive", 1, "1")
    track_editor_H_center:setField("groundType", 0, "ASPHALT")
    track_editor_H_center:registerObject("track_editor_H_center")
  end

  local track_editor_H_border = scenetree.findObject("track_editor_H_border")
  if not track_editor_H_border then
    track_editor_H_border = createObject("Material")
    track_editor_H_border.mapTo = "track_editor_H_border"
    track_editor_H_border:setField("diffuseColor", 0, "0.309392005 0.309388995 0.309388995 1")
    track_editor_H_border:setField("diffuseColor", 1, "0.850829005 0.0564090014 0 1")
    track_editor_H_border:setField("specularPower", 0, "50")
    track_editor_H_border:setField("specular", 0, "1 1 1 1")
    track_editor_H_border:setField("useAnisotropic", 0, "1")
    track_editor_H_border:setField("useAnisotropic", 1, "1")
    track_editor_H_border:setField("doubleSided", 0, "0")
    track_editor_H_border:setField("translucentBlendOp", 0, "None")
    track_editor_H_border:setField("materialTag0", 0, "beamng")
    track_editor_H_border:setField("materialTag1", 0, "RoadAndPath")
    track_editor_H_border:setField("colorMap", 0, "core/art/trackBuilder/track_editor_base_d.dds")
    track_editor_H_border:setField("colorMap", 1, "core/art/trackBuilder/track_editor_strip_raw_decal.dds")
    track_editor_H_border:setField("glow", 1, "1")
    track_editor_H_border:setField("emissive", 1, "1")
    track_editor_H_border:setField("groundType", 0, "ASPHALT")
    track_editor_H_border:registerObject("track_editor_H_border")
  end
end

M.loadCoreMaterials = function ()
  -- log('I','core', 'creating core materials....')
  createCommonMaterials()
  createTrackbuilderMaterials()

  -- then the new ones
  local subfiles = FS:findFiles("core/", '*.materials.json', -1, true, false)
  -- log('I','core', 'loadCoreMaterials json materials  = '..dumps(subfiles))
  -- log('I','core', 'Loading: ')
  for _, filename in ipairs(subfiles) do
    -- log('I','core', '      '..tostring(filename))
    loadJsonMaterialsFile(filename)
  end
end

local function createCommonMaterialData()
  TorqueScriptLua.setVar("$scroll", "1")
  TorqueScriptLua.setVar("$rotate", "2")
  TorqueScriptLua.setVar("$wave", "4")
  TorqueScriptLua.setVar("$scale", "8")
  TorqueScriptLua.setVar("$sequence", "16")

  local samplerClampLinear = createObject("GFXSamplerStateData")
  samplerClampLinear:setField("textureColorOp", 0, "GFXTOPModulate")
  samplerClampLinear:setField("addressModeU", 0, "GFXAddressClamp")
  samplerClampLinear:setField("addressModeV", 0, "GFXAddressClamp")
  samplerClampLinear:setField("addressModeW", 0, "GFXAddressClamp")
  samplerClampLinear:setField("magFilter", 0, "GFXTextureFilterLinear")
  samplerClampLinear:setField("minFilter", 0, "GFXTextureFilterLinear")
  samplerClampLinear:setField("mipFilter", 0, "GFXTextureFilterLinear")
  samplerClampLinear:registerObject("SamplerClampLinear")

  local samplerClampPoint = createObject("GFXSamplerStateData")
  samplerClampPoint:setField("textureColorOp", 0, "GFXTOPModulate")
  samplerClampPoint:setField("addressModeU", 0, "GFXAddressClamp")
  samplerClampPoint:setField("addressModeV", 0, "GFXAddressClamp")
  samplerClampPoint:setField("addressModeW", 0, "GFXAddressClamp")
  samplerClampPoint:setField("magFilter", 0, "GFXTextureFilterPoint")
  samplerClampPoint:setField("minFilter", 0, "GFXTextureFilterPoint")
  samplerClampPoint:setField("mipFilter", 0, "GFXTextureFilterPoint")
  samplerClampPoint:registerObject("SamplerClampPoint")

  local samplerWrapLinear = createObject("GFXSamplerStateData")
  samplerWrapLinear:setField("textureColorOp", 0, "GFXTOPModulate")
  samplerWrapLinear:setField("addressModeU", 0, "GFXTextureAddressWrap")
  samplerWrapLinear:setField("addressModeV", 0, "GFXTextureAddressWrap")
  samplerWrapLinear:setField("addressModeW", 0, "GFXTextureAddressWrap")
  samplerWrapLinear:setField("magFilter", 0, "GFXTextureFilterLinear")
  samplerWrapLinear:setField("minFilter", 0, "GFXTextureFilterLinear")
  samplerWrapLinear:setField("mipFilter", 0, "GFXTextureFilterLinear")
  samplerWrapLinear:registerObject("SamplerWrapLinear")


  local samplerWrapPoint = createObject("GFXSamplerStateData")
  samplerWrapPoint:setField("textureColorOp", 0, "GFXTOPModulate")
  samplerWrapPoint:setField("addressModeU", 0, "GFXTextureAddressWrap")
  samplerWrapPoint:setField("addressModeV", 0, "GFXTextureAddressWrap")
  samplerWrapPoint:setField("addressModeW", 0, "GFXTextureAddressWrap")
  samplerWrapPoint:setField("magFilter", 0, "GFXTextureFilterPoint")
  samplerWrapPoint:setField("minFilter", 0, "GFXTextureFilterPoint")
  samplerWrapPoint:setField("mipFilter", 0, "GFXTextureFilterPoint")
  samplerWrapPoint:registerObject("SamplerWrapPoint")
end

local function createShaderData()
  local particlesShaderData = createObject("ShaderData")
  particlesShaderData.DXVertexShaderFile = "shaders/common/particles/particlesV.hlsl"
  particlesShaderData.DXPixelShaderFile  = "shaders/common/particles/particlesP.hlsl"
  particlesShaderData.pixVersion = 5.0;
  particlesShaderData:registerObject("ParticlesShaderData")

  local offScreenShaderData = createObject("ShaderData")
  offScreenShaderData.DXVertexShaderFile = "shaders/common/particles/particleCompositeV.hlsl"
  offScreenShaderData.DXPixelShaderFile  = "shaders/common/particles/particleCompositeP.hlsl"
  offScreenShaderData.pixVersion = 5.0;
  offScreenShaderData:registerObject("OffscreenParticleCompositeShaderData")

  -----------------------------------------------------------------------------
  -- Planar Reflection
  -----------------------------------------------------------------------------
  local reflectBump = createObject("ShaderData")
  reflectBump.DXVertexShaderFile = "shaders/common/planarReflectBumpV.hlsl"
  reflectBump.DXPixelShaderFile = "shaders/common/planarReflectBumpP.hlsl"
  reflectBump:setField("samplerNames", 0, "$diffuseMap")
  reflectBump:setField("samplerNames", 1, "$refractMap")
  reflectBump:setField("samplerNames", 2, "$bumpMap")
  reflectBump.pixVersion = 5.0;
  reflectBump:registerObject("ReflectBump")

  local Reflect = createObject("ShaderData")
  Reflect.DXVertexShaderFile = "shaders/common/planarReflectV.hlsl"
  Reflect.DXPixelShaderFile = "shaders/common/planarReflectP.hlsl"
  Reflect:setField("samplerNames", 0, "$diffuseMap")
  Reflect:setField("samplerNames", 1, "$refractMap")
  Reflect.pixVersion = 5.0;
  Reflect:registerObject("Reflect")

  ------------------------------------------------------------------------------
  -- fxFoliageReplicator
  -----------------------------------------------------------------------------
  local fxFoliageReplicatorShader = createObject("ShaderData")
  fxFoliageReplicatorShader.DXVertexShaderFile = "shaders/common/fxFoliageReplicatorV.hlsl"
  fxFoliageReplicatorShader.DXPixelShaderFile = "shaders/common/fxFoliageReplicatorP.hlsl"
  fxFoliageReplicatorShader:setField("samplerNames", 0, "$diffuseMap")
  fxFoliageReplicatorShader:setField("samplerNames", 1, "$alphaMap")
  fxFoliageReplicatorShader.pixVersion = 1.4;
  fxFoliageReplicatorShader:registerObject("fxFoliageReplicatorShader")

  ------------------------------------------------------------------------------
  -- TerrainBlock
  -- Used when generating the blended base texture.
  -----------------------------------------------------------------------------
  local terrainBlendShader = createObject("ShaderData")
  terrainBlendShader.DXVertexShaderFile = "shaders/common/terrain/blendV.hlsl"
  terrainBlendShader.DXPixelShaderFile  = "shaders/common/terrain/blendP.hlsl"
  terrainBlendShader.pixVersion = 5.0;
  terrainBlendShader:registerObject("TerrainBlendShader")
end

local function createWaterShaderData()
  ---------------------------------------------------------------------------
  -- Water
  ---------------------------------------------------------------------------
  local waterShader = createObject("ShaderData")
  waterShader.DXVertexShaderFile = "shaders/common/water/waterV.hlsl"
  waterShader.DXPixelShaderFile  = "shaders/common/water/waterP.hlsl"
  waterShader.pixVersion = 5.0;
  waterShader:registerObject("WaterShader")

  local waterSampler = createObject("GFXSamplerStateData")
  waterSampler:setField("textureColorOp", 0, "GFXTOPModulate")
  waterSampler:setField("addressModeU", 0, "GFXAddressWrap")
  waterSampler:setField("addressModeV", 0, "GFXAddressWrap")
  waterSampler:setField("addressModeW", 0, "GFXAddressWrap")
  waterSampler:setField("magFilter", 0, "GFXTextureFilterLinear")
  waterSampler:setField("minFilter", 0, "GFXTextureFilterAnisotropic")
  waterSampler:setField("mipFilter", 0, "GFXTextureFilterLinear")
  waterSampler:setField("maxAnisotropy", 0, 4)
  waterSampler:registerObject("WaterSampler")

  local waterStateBlock = createObject("GFXStateBlockData")
  waterStateBlock.samplersDefined = true
  waterStateBlock.cullDefined = true
  waterStateBlock:setField("cullMode", 0, "GFXCullCCW")
  waterStateBlock:setField("samplerStates", 0, "WaterSampler")
  waterStateBlock:setField("samplerStates", 1, "SamplerClampPoint")
  waterStateBlock:setField("samplerStates", 2, "SamplerClampLinear")
  waterStateBlock:setField("samplerStates", 3, "SamplerClampPoint")
  waterStateBlock:setField("samplerStates", 4, "SamplerClampLinear")
  waterStateBlock:setField("samplerStates", 5, "SamplerClampLinear")
  waterStateBlock:setField("samplerStates", 6, "SamplerClampLinear")
  waterStateBlock:setField("samplerStates", 7, "SamplerClampPoint")
  waterStateBlock:registerObject("WaterStateBlock")

  local underWaterStateBlock = createObject("GFXStateBlockData")
  underWaterStateBlock:inheritParentFields(waterStateBlock)
  underWaterStateBlock:setField("cullMode", 0, "GFXCullCW")
  underWaterStateBlock:registerObject("UnderWaterStateBlock")

  local waterMat = createObject("CustomMaterial")
  waterMat:setField("sampler", "prepassTex", "#prepass[RT0]")
  waterMat:setField("sampler", "prepassDepthTex", "#prepass[Depth]")
  waterMat:setField("sampler", "reflectMap", "$reflectbuff")
  waterMat:setField("sampler", "refractBuff", "$backbuff")
  waterMat:setField("shader", 0, "WaterShader")
  waterMat:setField("stateBlock", 0, "WaterStateBlock")
  waterMat:setField("useAnisotropic", 0, "true")
  waterMat.version = 5.0
  waterMat:registerObject("WaterMat")

  ---------------------------------------------------------------------------
  -- Underwater
  ---------------------------------------------------------------------------
  local underWaterShader = createObject("ShaderData")
  underWaterShader.DXVertexShaderFile = "shaders/common/water/waterV.hlsl"
  underWaterShader.DXPixelShaderFile  = "shaders/common/water/waterP.hlsl"
  underWaterShader.defines  = "UNDERWATER"
  underWaterShader.pixVersion = 5.0;
  underWaterShader:registerObject("UnderWaterShader")

  local underwaterMat = createObject("CustomMaterial")
  underwaterMat:setField("sampler", "prepassTex", "#prepass[RT0]")
  underwaterMat:setField("sampler", "prepassDepthTex", "#prepass[Depth]")
  underwaterMat:setField("sampler", "refractBuff", "$backbuff")
  underwaterMat:setField("shader", 0, "UnderWaterShader")
  underwaterMat:setField("stateBlock", 0, "UnderWaterStateBlock")
  underwaterMat:setField("specular", 0, "0.75 0.75 0.75 1.0")
  underwaterMat.specularPower = 48.0
  underwaterMat.version = 5.0
  underwaterMat:registerObject("UnderwaterMat")

  ---------------------------------------------------------------------------
  -- Basic Water
  ---------------------------------------------------------------------------
  local waterBasicShader = createObject("ShaderData")
  waterBasicShader.DXVertexShaderFile = "shaders/common/water/waterBasicV.hlsl"
  waterBasicShader.DXPixelShaderFile  = "shaders/common/water/waterBasicP.hlsl"
  waterBasicShader.pixVersion = 5.0;
  waterBasicShader:registerObject("WaterBasicShader")

  local waterBasicStateBlock = createObject("GFXStateBlockData")
  waterBasicStateBlock.samplersDefined = true
  waterBasicStateBlock.cullDefined = true
  waterBasicStateBlock:setField("cullMode", 0, "GFXCullCCW")
  waterBasicStateBlock:setField("samplerStates", 0, "WaterSampler") -- noise
  waterBasicStateBlock:setField("samplerStates", 2, "SamplerClampLinear") -- $reflectbuff
  waterBasicStateBlock:setField("samplerStates", 3, "SamplerClampPoint")  -- $backbuff
  waterBasicStateBlock:setField("samplerStates", 4, "SamplerWrapLinear")  -- $cubemap
  waterBasicStateBlock:registerObject("WaterBasicStateBlock")

  local underWaterBasicStateBlock = createObject("GFXStateBlockData")
  underWaterStateBlock:inheritParentFields(waterBasicStateBlock)
  underWaterBasicStateBlock:setField("cullMode", 0, "GFXCullCW")
  underWaterBasicStateBlock:registerObject("UnderWaterBasicStateBlock")

  local waterBasicMat = createObject("CustomMaterial")
  waterBasicMat:setField("sampler", "reflectMap", "#reflectbuff")
  waterBasicMat:setField("sampler", "refractBuff", "$backbuff")
  waterBasicMat:setField("cubemap", 0, "NewLevelSkyCubemap")
  waterBasicMat:setField("shader", 0, "WaterBasicShader")
  waterBasicMat:setField("stateBlock", 0, "WaterBasicStateBlock")
  waterBasicMat.version = 5.0
  waterBasicMat:registerObject("WaterBasicMat")

  ---------------------------------------------------------------------------
  -- Basic UnderWater
  ---------------------------------------------------------------------------
  local underWaterBasicShader = createObject("ShaderData")
  underWaterBasicShader.DXVertexShaderFile = "shaders/common/water/waterBasicV.hlsl"
  underWaterBasicShader.DXPixelShaderFile  = "shaders/common/water/waterBasicP.hlsl"
  underWaterBasicShader.defines = "UNDERWATER";
  underWaterBasicShader.pixVersion = 5.0;
  underWaterBasicShader:registerObject("UnderWaterBasicShader")

  local underwaterBasicMat = createObject("CustomMaterial")
  underwaterBasicMat:setField("sampler", "refractBuff", "$backbuff")
  underwaterBasicMat:setField("shader", 0, "UnderWaterBasicShader")
  underwaterBasicMat:setField("stateBlock", 0, "UnderWaterBasicStateBlock")
  underwaterBasicMat.version = 5.0
  underwaterBasicMat:registerObject("UnderwaterBasicMat")
end

local function createScatterSkyData()
  local scatterSkySBData = createObject("GFXStateBlockData")
  scatterSkySBData.samplersDefined = true
  scatterSkySBData.cullDefined = true
  scatterSkySBData.zDefined = true
  scatterSkySBData.zEnable = true
  scatterSkySBData.zWriteEnable = false
  scatterSkySBData.vertexColorEnable = true
  if TorqueScriptLua.getBoolVar("$Scene::useReversedDepthBuffer") then
    scatterSkySBData:setField("zFunc", 0, "GFXCmpGreaterEqual")
  else
    scatterSkySBData:setField("zFunc", 0, "GFXCmpLessEqual")
  end
  scatterSkySBData:setField("samplerStates", 0, "SamplerClampLinear")
  scatterSkySBData:setField("samplerStates", 1, "SamplerClampLinear")
  scatterSkySBData:setField("cullMode", 0, "GFXCullNone")
  scatterSkySBData:registerObject("ScatterSkySBData")

  local scatterSkyShaderData = createObject("ShaderData")
  scatterSkyShaderData.DXVertexShaderFile = "shaders/common/scatterSkyV.hlsl"
  scatterSkyShaderData.DXPixelShaderFile  = "shaders/common/scatterSkyP.hlsl"
  scatterSkyShaderData.pixVersion = 5.0;
  scatterSkyShaderData:registerObject("ScatterSkyShaderData")
end

local function createCloudsData()
  ------------------------------------------------------------------------------
  -- CloudLayer
  ------------------------------------------------------------------------------
  local cloudLayerShader = createObject("ShaderData")
  cloudLayerShader.DXVertexShaderFile = "shaders/common/cloudLayerV.hlsl"
  cloudLayerShader.DXPixelShaderFile  = "shaders/common/cloudLayerP.hlsl"
  cloudLayerShader.pixVersion = 5.0;
  cloudLayerShader:registerObject("CloudLayerShader")

  ------------------------------------------------------------------------------
  -- BasicClouds
  ------------------------------------------------------------------------------
  local basicCloudsShader = createObject("ShaderData")
  basicCloudsShader.DXVertexShaderFile = "shaders/common/basicCloudsV.hlsl"
  basicCloudsShader.DXPixelShaderFile  = "shaders/common/basicCloudsP.hlsl"
  basicCloudsShader.pixVersion = 5.0;
  basicCloudsShader:registerObject("BasicCloudsShader")
end


local function createVehicleData()
  if not scenetree.findObject("PropSelectionCustomMat") then
    local shader = createObject("ShaderData")
    shader.DXVertexShaderFile = "shaders/common/vehicle/propselection.hlsl"
    shader.DXPixelShaderFile  = "shaders/common/vehicle/propselection.hlsl"
    shader.pixVersion = 5.0;
    shader:registerObject("PropSelectionShader")

    local stateBlock = createObject("GFXStateBlockData")
    stateBlock.blendDefined = true
    stateBlock.blendEnable = true
    stateBlock:setField("blendSrc", 0, "GFXBlendSrcAlpha")
    stateBlock:setField("blendDest", 0, "GFXBlendInvSrcAlpha")
    stateBlock.zDefined = true
    stateBlock.zEnable = true
    stateBlock.zWriteEnable = false
    if TorqueScriptLua.getBoolVar("$Scene::useReversedDepthBuffer") then
      stateBlock.zBias = 1
      stateBlock.zSlopeBias = 1
    else
      stateBlock.zBias = -5
      stateBlock.zSlopeBias = -5
    end
    stateBlock:registerObject("ProSelectionStateBlock")

    local waterBasicMat = createObject("CustomMaterial")
    waterBasicMat:setField("shader", 0, "PropSelectionShader")
    waterBasicMat:setField("stateBlock", 0, "ProSelectionStateBlock")
    waterBasicMat:registerObject("PropSelectionCustomMat")
  end
end

M.initializeCore = function()
  -- log("I", "core", "initializeCore called.....")

  if M.initialized == true then return end

  -- Seed the random number generator.
  setRandomSeed(-1)

  require("client/objectsRequiredForStartup")

  -- Initialize the canvas.
  local canvas_module = require("client/canvas") -- Very basic functions used by everyone.
  canvas_module.initializeCanvas()

  -- Materials and Shaders for rendering various object types
  M.loadCoreMaterials()

  createCommonMaterialData()
  createShaderData()
  createWaterShaderData()
  createScatterSkyData()
  createCloudsData()
  createVehicleData()

  M.initialized = true
end

M.reloadCore = function()
  -- Use our prefs to configure our Canvas/Window
  local canvas_module = require("client/canvas") -- Very basic functions used by everyone.

  -- Initialize the canvas.
  canvas_module.initializeCanvas()

  M.initialized = true
end

return M
