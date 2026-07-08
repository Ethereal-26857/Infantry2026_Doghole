#include "ins_ui.hpp"

#include "ins_all.hpp"

static hello_world::referee::UiMgr unique_ui_mgr;
static bool is_ui_mgr_inited = false;

static robot::UiDrawer unique_ui_drawer;
static bool is_ui_drawer_inited = false;

hello_world::referee::UiMgr* GetUiMgrIns(void)
{
  if (!is_ui_mgr_inited) {
    unique_ui_mgr.registerReferee(GetRfrIns());
    is_ui_mgr_inited = true;
  }
  return &unique_ui_mgr;
}

robot::UiDrawer* GetUiDrawerIns(void)
{
  if (!is_ui_drawer_inited) {
    unique_ui_drawer.initUiDrawer(GetUiMgrIns());
    is_ui_drawer_inited = true;
  }
  return &unique_ui_drawer;
}
