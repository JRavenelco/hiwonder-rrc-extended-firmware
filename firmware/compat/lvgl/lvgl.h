/* Case-sensitivity shim. The vendor sources include "lvgl/lvgl.h", but the
 * directory is named Third_Party/LVGL: that only resolves on a case-insensitive
 * file system (Windows). This forwarding header makes it resolve everywhere
 * without renaming or editing any vendor file. */
#include "../../Third_Party/LVGL/lvgl.h"
