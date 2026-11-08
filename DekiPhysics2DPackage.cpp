// Package entry point for deki-physics-2d.
#include "DekiPhysics2DAPI.h"
#include <deki/interop/Plugin.h>
#include <deki/LogSystem.h>

extern void DekiPhysics2DRegisterComponents();
extern int DekiPhysics2DGetAutoComponentCount();
extern const Deki::ComponentMeta* DekiPhysics2DGetAutoComponentMeta(int index);

#ifdef DEKI_EDITOR

static bool s_Physics2DRegistered = false;

extern "C"
{
    DEKI_PHYSICS2D_API int DekiPhysics2DEnsureRegistered(void)
    {
        if (s_Physics2DRegistered)
        {
            return ::DekiPhysics2DGetAutoComponentCount();
        }
        s_Physics2DRegistered = true;
        ::DekiPhysics2DRegisterComponents();
        return ::DekiPhysics2DGetAutoComponentCount();
    }

    DEKI_PLUGIN_API const char* DekiPluginGetName(void)
    {
        return "Deki Physics 2D Package";
    }
    DEKI_PLUGIN_API const char* DekiPluginGetVersion(void)
    {
#ifdef DEKI_PACKAGE_VERSION
        return DEKI_PACKAGE_VERSION;
#else
        return "0.0.0-dev";
#endif
    }
    DEKI_PLUGIN_API int DekiPluginInit(void)
    {
        return 0;
    }
    DEKI_PLUGIN_API void DekiPluginShutdown(void)
    {
        s_Physics2DRegistered = false;
    }
    DEKI_PLUGIN_API int DekiPluginGetComponentCount(void)
    {
        return ::DekiPhysics2DGetAutoComponentCount();
    }
    DEKI_PLUGIN_API const Deki::ComponentMeta* DekiPluginGetComponentMeta(int index)
    {
        return ::DekiPhysics2DGetAutoComponentMeta(index);
    }
    DEKI_PLUGIN_API void DekiPluginRegisterComponents(void)
    {
        DekiPhysics2DEnsureRegistered();
    }
}  // extern "C"

#endif  // DEKI_EDITOR
