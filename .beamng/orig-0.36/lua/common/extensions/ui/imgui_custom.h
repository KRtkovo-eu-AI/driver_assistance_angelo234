// feel free to add things here
typedef unsigned __int64 size_t;

//typedef void (*ImGuiSizeCallback)(ImGuiSizeCallbackData* data);

typedef const struct ImTextureID_type & ImTextureID;
typedef struct ImTextureHandler {
  const void* ptr_do_not_use;
} ImTextureHandler;
void imgui_ImTextureHandler_set(const ImTextureHandler &hnd, const char *path);
ImTextureID imgui_ImTextureHandler_get(const ImTextureHandler &hnd);
void imgui_ImTextureHandler_size(const ImTextureHandler &hnd, const ImVec2 &vec2);
const char* imgui_ImTextureHandler_format(const ImTextureHandler &hnd);
bool imgui_ImTextureHandler_isCached(const char* path);

const ImGuiContext& imgui_GetMainContext();
void imgui_registerDrawData( int queueId);
void imgui_NewFrame2( int queueId);

//typedef struct voidPtr * voidPtr;
//typedef int (*ImGuiInputTextCallback)(ImGuiInputTextCallbackData *data);

bool imgui_PushFont2( unsigned int idx);
bool imgui_PushFont3(const char* uniqueId);
int imgui_IoFontsGetCount();
void imgui_TextGlyph(unsigned int unicode);
const char* imgui_IoFontsGetName( unsigned int idx);
void imgui_SetDefaultFont( unsigned int idx);

bool imgui_InputText( const char* label, const char& buf, size_t buf_size, ImGuiInputTextFlags flags, const void* callback, const void* user_data);
bool imgui_InputTextMultiline( const char* label, const char& buf, size_t buf_size, const ImVec2& size, ImGuiInputTextFlags flags, const void* callback, const void* user_data);
//void imgui_PlotMultiLines( const char* label, int num_datas, const char* names[], const ImColor* colors, float** datas, int values_count, const char* overlay_text, float scale_min, float scale_max, ImVec2 graph_size);
//void imgui_PlotMultiHistograms( const char* label, int num_hists, const char** names, const ImColor* colors, float** datas, int values_count, const char* overlay_text, float scale_min, float scale_max, ImVec2 graph_size, bool sumValues);

/*
struct PlotMulti2Options {
    const char* label;
    int num_datas;
    const char* names[];
    const ImColor* colors;
    void *getter;
    const float* datas[];
    int values_count;
    const char* overlay_text;
    float scale_min[2];
    float scale_max[2];
    ImVec2 graph_size;
    bool sumValues; //only for Histograms
    const char* axis_text[3];
    int &num_format;
    int axis_format[2];
    bool grid_x;
    bool display_legend;
    bool display_legend_last_value;
    float background_alpha;
};
void imgui_PlotMulti2Lines(  struct PlotMulti2Options &options);
void imgui_PlotMulti2Histograms( struct PlotMulti2Options &options);
*/

ImU32 imgui_GetColorU32ByVec4(const ImVec4& col);

void imgui_DockBuilderDockWindow(const char* window_name, ImGuiID node_id);
void imgui_DockBuilderAddNode(ImGuiID node_id, ImVec2 ref_size, ImGuiDockNodeFlags flags);
ImGuiID imgui_DockBuilderSplitNode( ImGuiID node_id, ImGuiDir split_dir, float size_ratio_for_node_at_dir, const ImGuiID& out_id_dir, const ImGuiID& out_id_other);
void imgui_DockBuilderFinish(ImGuiID node_id);

void imgui_BeginDisabled( bool disable);
void imgui_EndDisabled();

const char * imgui_TextFilter_GetInputBuf(const ImGuiTextFilter& ImGuiTextFilter_ctx);
void imgui_TextFilter_SetInputBuf(const ImGuiTextFilter& ImGuiTextFilter_ctx, const char * text);
int imgui_getMonitorIndex();
bool imgui_getCurrentMonitorSize(const ImVec2& vec2);

void imgui_LoadIniSettingsFromDisk(const char* ini_filename);
void imgui_SaveIniSettingsToDisk(const char* ini_filename);
void imgui_ClearActiveID();

// TextEditor below
typedef struct TextEditor TextEditor;
const TextEditor& imgui_createTextEditor();
void imgui_destroyTextEditor(const TextEditor& te);
void imgui_TextEditor_SetLanguageDefinition(const TextEditor& te, const char* str);
void imgui_TextEditor_Render(const TextEditor& te, const char* title, const ImVec2& size, bool border);
void imgui_TextEditor_SetText(const TextEditor& te, const char* text);
const char *imgui_TextEditor_GetText(const TextEditor& te);
bool imgui_TextEditor_IsTextChanged(const TextEditor& te);

// Imgui Knob plugin
typedef int ImGuiKnobFlags;

enum ImGuiKnobFlags_ {
    ImGuiKnobFlags_NoTitle = 1 << 0,
    ImGuiKnobFlags_NoInput = 1 << 1,
    ImGuiKnobFlags_ValueTooltip = 1 << 2,
    ImGuiKnobFlags_DragHorizontal = 1 << 3,
};

typedef int ImGuiKnobVariant;
enum ImGuiKnobVariant_ {
    ImGuiKnobVariant_Tick = 1 << 0,
    ImGuiKnobVariant_Dot = 1 << 1,
    ImGuiKnobVariant_Wiper = 1 << 2,
    ImGuiKnobVariant_WiperOnly = 1 << 3,
    ImGuiKnobVariant_WiperDot = 1 << 4,
    ImGuiKnobVariant_Stepped = 1 << 5,
    ImGuiKnobVariant_Space = 1 << 6,
};

bool imgui_Knob(const char *label, const float &p_value, float v_min, float v_max, float speed, const char *format, ImGuiKnobVariant variant, float size, ImGuiKnobFlags flags, int steps);
bool imgui_KnobInt(const char *label, const int &p_value, int v_min, int v_max, float speed, const char *format, ImGuiKnobVariant variant, float size, ImGuiKnobFlags flags, int steps);

// ##### ##### #####
// ##### ##### #####
// ##### ##### #####
typedef struct FFIBool {
  bool value;
} FFIBool;
typedef struct FFIFloat {
  float value;
} FFIFloat;
typedef struct FFIFloat2 {
  float x;
  float y;
} FFIFloat2;
typedef struct FFIString {
  const char* value;
  int size;
} FFIString;

void imgui_AllocString(const FFIString& obj, int size, const char* str);
void imgui_FreeString(const FFIString& obj);

void imgui_ShowDemoWindow_Test(const FFIBool& p_open);
bool imgui_CheckboxTest(const char& label, const FFIBool& v);
bool imgui_SliderFloatTest(const char* label, const FFIFloat& v, float v_min, float v_max, const char* format, ImGuiSliderFlags flags);
bool imgui_SliderFloat2Test(const char* label, const FFIFloat2& v, float v_min, float v_max, const char* format, ImGuiSliderFlags flags);
bool imgui_InputTextTest(const char* label, FFIString buf, size_t buf_size, ImGuiInputTextFlags flags, const void* callback, const void* user_data);
