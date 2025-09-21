-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local al_ShadowVisualizeShader = scenetree.findObject("AL_ShadowVisualizeShader")
if not al_ShadowVisualizeShader then
  al_ShadowVisualizeShader = createObject("ShaderData")
  al_ShadowVisualizeShader.DXVertexShaderFile  = "shaders/common/guiMaterialV.hlsl"
  al_ShadowVisualizeShader.DXPixelShaderFile   = "shaders/common/lighting/advanced/dbgShadowVisualizeP.hlsl"
  al_ShadowVisualizeShader.pixVersion = 5.0;
  al_ShadowVisualizeShader:registerObject("AL_ShadowVisualizeShader")
end

local al_ShadowVisualizeMaterial = scenetree.findObject("AL_ShadowVisualizeMaterial")
if not al_ShadowVisualizeMaterial then
  al_ShadowVisualizeMaterial = createObject("CustomMaterial")
  al_ShadowVisualizeMaterial:setField("shader", 0, "AL_ShadowVisualizeShader")
  al_ShadowVisualizeMaterial:setField("stateBlock", 0, "AL_DepthVisualizeState")
  al_ShadowVisualizeMaterial:setField("sampler", "shadowMap", "#AL_ShadowVizTexture")
  al_ShadowVisualizeMaterial:setField("sampler", "depthViz", "depthViz")
  al_ShadowVisualizeMaterial.pixVersion = 5.0;
  al_ShadowVisualizeMaterial:registerObject("AL_ShadowVisualizeMaterial")
end

--[[
singleton GuiControlProfile( AL_ShadowLabelTextProfile )
{
    fontColor = "0 0 0";
    autoSizeWidth = true;
    autoSizeHeight = true;
    justify = "left";
    fontSize = 14;
};

/// Toggles the visualization of the pre-pass lighting buffer.
function toggleShadowViz()
{
    if ( AL_ShadowVizOverlayCtrl.isAwake() )
    {
        setShadowVizLight( 0 );
        Canvas.popDialog( AL_ShadowVizOverlayCtrl );
    }
    else
    {
        Canvas.pushDialog( AL_ShadowVizOverlayCtrl, 100 );
        _setShadowVizLight( EWorldEditor.getSelectedObject( 0 ) );
    }
}

/// Called from the WorldEditor when an object is selected.
function _setShadowVizLight( %light, %force )
{
    if ( !AL_ShadowVizOverlayCtrl.isAwake() )
        return;

    if ( AL_ShadowVizOverlayCtrl.isLocked && !%force )
        return;

    // Resolve the object to the client side.
    if ( isObject( %light ) )
    {
        %clientLight = %light;
        %sizeAndAspect = setShadowVizLight( %clientLight );
    }

    AL_ShadowVizOverlayCtrl-->MatCtrl.setMaterial( "AL_ShadowVisualizeMaterial" );

    %text = "ShadowViz";
    if ( isObject( %light ) )
        %text = %text @ " : " @ getWord( %sizeAndAspect, 0 ) @ " x " @ getWord( %sizeAndAspect, 1 );

    AL_ShadowVizOverlayCtrl-->WindowCtrl.text = %text;
}

/// For convenience, push the viz dialog and set the light manually from the console.
function showShadowVizForLight( %light )
{
    if ( !AL_ShadowVizOverlayCtrl.isAwake() )
        Canvas.pushDialog( AL_ShadowVizOverlayCtrl, 100 );
    _setShadowVizLight( %light, true );
}

// Prevent shadowViz from changing lights in response to editor selection
// events until unlock is called. The only way a vis light will change while locked
// is if showShadowVizForLight is explicitly called by the user.
function lockShadowViz()
{
    AL_ShadowVizOverlayCtrl.islocked = true;
}

function unlockShadowViz()
{
    AL_ShadowVizOverlayCtrl.islocked = false;
}
]]