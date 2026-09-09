// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0.
#include "ai_sapiens_mujoco/mujoco_viewer_ui.hpp"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <algorithm>
#include <cmath>
#include "ai_sapiens_mujoco/viewer_scene_styling.hpp"
namespace ai_sapiens_mujoco {
void MujocoViewerUi::initialize(mjvOption * option, bool gantry_present) {
  if (initialized_) return;
  option_ = option; gantry_ = gantry_present;
  IMGUI_CHECKVERSION(); ImGui::CreateContext();
  auto & io = ImGui::GetIO(); io.IniFilename = nullptr;
  io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",16.0f);
  bold_ = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",16.0f);
  ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
  ImGui_ImplOpenGL3_Init("#version 130");
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(15, 15);
    style.WindowRounding = 5.0f;
    style.FramePadding = ImVec2(5, 5);
    style.FrameRounding = 4.0f;
    style.ItemSpacing = ImVec2(12, 4);
    style.ItemInnerSpacing = ImVec2(8, 6);
    style.IndentSpacing = 25.0f;
    style.ScrollbarSize = 15.0f;
    style.ScrollbarRounding = 9.0f;
    style.GrabMinSize = 5.0f;
    style.GrabRounding = 3.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.105f, 0.11f, 1.0f);
    style.Colors[ImGuiCol_Header] = ImVec4(0.2f, 0.205f, 0.21f, 1.0f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.3f, 0.305f, 0.31f, 1.0f);
    style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.15f, 0.1505f, 0.151f, 1.0f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.2f, 0.205f, 0.21f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.3f, 0.305f, 0.31f, 1.0f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.15f, 0.1505f, 0.151f, 1.0f);
  initialized_ = true;
}
void MujocoViewerUi::shutdown() {
  if (!initialized_) return;
  ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext();
  initialized_ = false;
}
void MujocoViewerUi::resize(int w,int h,mjrContext *) {width_=w; height_=h;}
mjrRect MujocoViewerUi::scene_viewport() const {return {0,0,width_,height_};}
bool MujocoViewerUi::cursor_is_in_scene() const {
  return initialized_ && !ImGui::GetIO().WantCaptureMouse;
}
ViewerUiResult MujocoViewerUi::handle_event(const ViewerUiEvent & e,const mjrContext *) {
  ViewerUiResult result;
  if (!initialized_) return result;
  const auto & io=ImGui::GetIO();
  result.handled=e.type==mjEVENT_KEY ? io.WantCaptureKeyboard : io.WantCaptureMouse;
  if(e.type==mjEVENT_KEY && !result.handled) {
    if(e.key==GLFW_KEY_F1){help_=!help_;result.handled=true;}
    if(e.key==GLFW_KEY_F2){info_=!info_;result.handled=true;}
    if(e.key==GLFW_KEY_BACKSPACE) result.action=ViewerUiAction::kReset;
    if(e.key==GLFW_KEY_SPACE) result.action=ViewerUiAction::kPause;
    if(e.key==GLFW_KEY_A && e.control) { result.action=ViewerUiAction::kAlign; return result; }
    if(e.key==GLFW_KEY_A && gantry_) result.action=ViewerUiAction::kAttachGantry;
    if(e.key==GLFW_KEY_R && gantry_) result.action=ViewerUiAction::kReleaseGantry;
    const int keys[]={GLFW_KEY_V,GLFW_KEY_G,GLFW_KEY_C,GLFW_KEY_F,GLFW_KEY_I,GLFW_KEY_M};
    mjtByte * values[]={option_->geomgroup+kVisualGeomGroup,option_->geomgroup+kCollisionGeomGroup,
      option_->flags+mjVIS_CONTACTPOINT,option_->flags+mjVIS_CONTACTFORCE,
      option_->flags+mjVIS_INERTIA,option_->flags+mjVIS_COM};
    for(int i=0;i<6;++i) if(e.key==keys[i]){*values[i]=!*values[i];result.handled=true;}
  }
  return result;
}
void MujocoViewerUi::update_status(double fps,int contacts,bool,const char *,bool attached,double height,const mjrContext *) {
  fps_=fps;contacts_=contacts;attached_=attached;gantry_height_=height;
}
ViewerUiAction MujocoViewerUi::take_action(){auto a=pending_;pending_=ViewerUiAction::kNone;return a;}
void MujocoViewerUi::render(const mjrContext *) {
  if(!initialized_ || width_<=0 || height_<=0) return;
  ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
  if(ImGui::BeginMainMenuBar()) {
    if(ImGui::BeginMenu("Menu")) {
      ImGui::MenuItem("Info & Visualization",nullptr,&left_);
      ImGui::MenuItem("Control",nullptr,&right_);
      if (ImGui::MenuItem("Velocity Panel",nullptr,&velocity_panel_)) right_ = true;
      if (ImGui::MenuItem("Mimic Panel",nullptr,&mimic_panel_)) right_ = true;
      ImGui::MenuItem("Ground Reaction Force", nullptr, false, false);
      ImGui::EndMenu();
    } ImGui::EndMainMenuBar();
  }
  const auto * vp=ImGui::GetMainViewport();
  const auto flags=ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse;
  auto check=[](const char * name,mjtByte & value){bool v=value!=0;if(ImGui::Checkbox(name,&v))value=v;};
  if(left_) {
    ImGui::SetNextWindowPos(vp->WorkPos);ImGui::SetNextWindowSize(ImVec2(300,vp->WorkSize.y));
    ImGui::Begin("Info & Visualization",&left_,flags);
    auto hint = [](const char * text) {
      ImGui::SameLine(); ImGui::TextDisabled("(?)");
      if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip(); ImGui::PushTextWrapPos(ImGui::GetFontSize()*22);
        ImGui::TextUnformatted(text); ImGui::PopTextWrapPos(); ImGui::EndTooltip();
      }
    };
    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const float available = ImGui::GetContentRegionAvail().x;
    const float half = (available-gap)*0.5f;
    const bool enabled = controls_ && controls_->connected && !controls_->busy;
    if (bold_) ImGui::PushFont(static_cast<ImFont*>(bold_));
    ImGui::TextUnformatted("K1 Simulation");
    if (bold_) ImGui::PopFont();
    ImGui::TextColored(controls_ && controls_->connected ? ImVec4(.42f,.78f,.55f,1) : ImVec4(.9f,.65f,.3f,1),
      "%s",controls_ ? controls_->mode.c_str() : "Disconnected");
    if (controls_) {
      hint(controls_->note.c_str());
      if (!controls_->connected || controls_->busy) ImGui::TextWrapped("%s",controls_->note.c_str());
    }
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Simulation",ImGuiTreeNodeFlags_DefaultOpen)) {
      const float third = (available-2*gap)/3;
      ImGui::BeginDisabled(!enabled);
      if (ImGui::Button("Reset",ImVec2(third,0))) pending_=ViewerUiAction::kReset;
      ImGui::SameLine();
      if (ImGui::Button(controls_ && controls_->paused ? "Run" : "Pause",ImVec2(third,0))) pending_=ViewerUiAction::kPause;
      ImGui::EndDisabled(); ImGui::SameLine();
      if (ImGui::Button("Align",ImVec2(third,0))) pending_=ViewerUiAction::kAlign;
      ImGui::BeginDisabled(!enabled);
      if (ImGui::Button("Walkready",ImVec2(half,0))) pending_=ViewerUiAction::kReadyPose;
      ImGui::SameLine();
      if (ImGui::Button("Damping",ImVec2(half,0))) pending_=ViewerUiAction::kDamping;
      ImGui::EndDisabled();
    }
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Physics",ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Spacing();
      ImGui::TextUnformatted("Floor friction");
      hint("Sliding coefficient (0.05-2.0). Applies immediately to floor contacts. Reset friction restores the original contact settings.");
      if (physics_.floor_available) {
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderFloat("##friction",&friction_,0.05f,2.0f,"%.2f")) pending_=ViewerUiAction::kSetFriction;
        ImGui::TextDisabled("Default %.2f",physics_.nominal_friction);
        ImGui::SameLine(available-ImGui::CalcTextSize("Reset friction").x-ImGui::GetStyle().FramePadding.x*2+ImGui::GetStyle().WindowPadding.x);
        if (ImGui::SmallButton("Reset friction")) pending_=ViewerUiAction::kResetFriction;
      } else ImGui::TextDisabled("Floor unavailable");
      ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
      ImGui::TextUnformatted("Payload (kg)");
      hint("Additional point mass at the base center of mass (0-100 kg). Enter a value, then Apply. Simulation Reset preserves physics settings.");
      if (physics_.payload_available) {
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputFloat("##payload",&payload_,0.5f,1.0f,"%.2f"))
          payload_=std::isfinite(payload_) ? std::clamp(payload_,0.0f,100.0f) : 0.0f;
        if (ImGui::Button("Apply payload",ImVec2(half,0))) pending_=ViewerUiAction::kSetPayload;
        ImGui::SameLine();
        if (ImGui::Button("Reset payload",ImVec2(half,0))) {pending_=ViewerUiAction::kResetPayload;payload_=0;}
        ImGui::TextDisabled("Applied  +%.2f kg",physics_.current_mass-physics_.nominal_mass);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s: %.3f kg nominal / %.3f kg total",physics_.base_body.c_str(),physics_.nominal_mass,physics_.current_mass);
      } else ImGui::TextDisabled("Base unavailable");
      ImGui::Spacing();
    }
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Visualization")) {
      ImGui::TextUnformatted("Frame"); ImGui::SetNextItemWidth(-1);
      const char * frames[]={"None","Body","Geom","Site","Camera","Light","Contact","World"};
      ImGui::Combo("##frame",&option_->frame,frames,8);
      if (ImGui::BeginTable("Visual options",2,ImGuiTableFlags_SizingStretchSame)) {
        const char * labels[]={"Hull","Joints","Actuators","Inertia","Contacts","Forces","Transparent","CoM","Selection"};
        const int indices[]={mjVIS_CONVEXHULL,mjVIS_JOINT,mjVIS_ACTUATOR,mjVIS_INERTIA,mjVIS_CONTACTPOINT,mjVIS_CONTACTFORCE,mjVIS_TRANSPARENT,mjVIS_COM,mjVIS_SELECT};
        for (int i=0;i<9;++i) {ImGui::TableNextColumn();check(labels[i],option_->flags[indices[i]]);}
        ImGui::EndTable();
      }
      ImGui::Separator(); ImGui::TextDisabled("Geometry groups");
      for(int i=0;i<6;++i){char label[4];snprintf(label,sizeof(label),"G%d",i);check(label,option_->geomgroup[i]);if(i%3!=2)ImGui::SameLine();}
    }
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Help & Diagnostics")) {
      ImGui::MenuItem("Help overlay","F1",&help_);ImGui::MenuItem("Info overlay","F2",&info_);
      ImGui::Separator();
      ImGui::TextWrapped("Push robot: Ctrl + right-drag");
      ImGui::TextWrapped("Reset: Backspace / Pause: Space / Align: Ctrl+A");
    }
    ImGui::Spacing();ImGui::Separator();
    ImGui::TextDisabled("%.0f FPS    /    %d contacts",fps_,contacts_);
    ImGui::End();
  }
  if(right_) {
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x+vp->WorkSize.x-500,vp->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(500,vp->WorkSize.y));ImGui::Begin("Control",&right_,flags);
    if(gantry_ && ImGui::CollapsingHeader("Gantry",ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("%s | Height: %.3f m",attached_?"Attached":"Released",gantry_height_);
      ImGui::BeginDisabled(!attached_);
      if(ImGui::Button("Raise +2 cm"))pending_=ViewerUiAction::kRaiseGantry;
      ImGui::SameLine();if(ImGui::Button("Lower -2 cm"))pending_=ViewerUiAction::kLowerGantry;
      if(ImGui::Button("Release robot"))pending_=ViewerUiAction::kReleaseGantry;
      ImGui::EndDisabled();ImGui::SameLine();ImGui::BeginDisabled(attached_);
      if(ImGui::Button("Attach robot"))pending_=ViewerUiAction::kAttachGantry;
      ImGui::EndDisabled();
    }
    if (controls_) {
      ImGui::BeginDisabled(!controls_->connected || controls_->busy);
      if (velocity_panel_ && ImGui::CollapsingHeader("Velocity Panel",ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped("Policy: %s",controls_->velocity_asset.c_str());
        if (ImGui::Button("Start Velocity")) pending_=ViewerUiAction::kVelocity;
        ImGui::SameLine();
        if (ImGui::Button("Stop Velocity")) pending_=ViewerUiAction::kDamping;
        ImGui::TextDisabled("Normalized input [-1, 1]; scaled by policy ranges");
        ImGui::BeginDisabled(controls_->mode != "Velocity" || controls_->paused);
        ImGui::SliderFloat("Forward / Back",&controls_->velocity[0],-1,1,"%.2f");
        ImGui::SliderFloat("Left / Right",&controls_->velocity[1],-1,1,"%.2f");
        ImGui::SliderFloat("Turn",&controls_->velocity[2],-1,1,"%.2f");
        if (ImGui::Button("Zero velocity")) pending_=ViewerUiAction::kStopVelocity;
        ImGui::EndDisabled();
      }
      if (mimic_panel_ && ImGui::CollapsingHeader("Mimic Panel",ImGuiTreeNodeFlags_DefaultOpen)) {
        for (size_t i = 0; i < controls_->motions.size(); ++i) {
          const auto & motion = controls_->motions[i];
          const bool running = controls_->mode == motion.name;
          const bool can_run = controls_->mode == "Velocity" && !controls_->paused;
          ImGui::PushID(static_cast<int>(i));
          if (running) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.42f, 0.24f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.52f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.12f, 0.32f, 0.18f, 1.0f));
          }
          const std::string label = running ? "Stop " + motion.name : motion.name;
          ImGui::BeginDisabled(!running && !can_run);
          if (ImGui::Button(label.c_str(), ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
            controls_->selected_motion = static_cast<int>(i);
            pending_ = running ? ViewerUiAction::kVelocity : ViewerUiAction::kMimic;
          }
          ImGui::EndDisabled();
          if (running) ImGui::PopStyleColor(3);
          ImGui::PopID();
        }
        if (controls_->motions.empty()) ImGui::TextDisabled("No mimic motions configured.");
        ImGui::TextDisabled("Select a motion in Velocity. Stop returns to Velocity.");
      }
      ImGui::EndDisabled();
    } else {
      ImGui::TextWrapped("Policy controls unavailable. Check viewer startup log.");
    }
    ImGui::End();
  }
  const auto overlay_flags=ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_AlwaysAutoResize|
    ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_NoFocusOnAppearing|ImGuiWindowFlags_NoNav;
  if(help_) {
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x+(left_?300:0)+10,vp->WorkPos.y+10));
    ImGui::SetNextWindowBgAlpha(.35f);ImGui::Begin("Help Overlay",&help_,overlay_flags);
    ImGui::TextUnformatted("CONTROLS");ImGui::Separator();
    ImGui::TextUnformatted("Camera: mouse drag / scroll\nPush body: Ctrl + right drag\nGantry: Up / Down / A / R\nReset: Backspace    Align: Ctrl+A\nPause / Run: Space\nHelp: F1    Info: F2");ImGui::End();
  }
  if(info_) {
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x+(left_?300:0)+10,vp->WorkPos.y+vp->WorkSize.y-10),ImGuiCond_Always,ImVec2(0,1));
    ImGui::SetNextWindowBgAlpha(.35f);ImGui::Begin("Info Overlay",&info_,overlay_flags);
    ImGui::Text("FPS: %.1f\nContacts: %d",fps_,contacts_);ImGui::End();
  }
  ImGui::Render();ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
}  // namespace ai_sapiens_mujoco
