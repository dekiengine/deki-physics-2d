#pragma once

// Defines only DEKI_PHYSICS2D_API. Package headers include this rather than
// the package's umbrella header.

#ifdef DEKI_EDITOR
#ifdef _WIN32
#ifdef DEKI_PHYSICS2D_EXPORTS
#define DEKI_PHYSICS2D_API __declspec(dllexport)
#else
#define DEKI_PHYSICS2D_API __declspec(dllimport)
#endif
#else
#define DEKI_PHYSICS2D_API
#endif
#else
#define DEKI_PHYSICS2D_API
#endif
