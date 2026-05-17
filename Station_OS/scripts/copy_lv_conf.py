"""
Pre-build script: copy project header files into library directories.

1. lv_conf.h  → lvgl/   so LVGL v9 finds it via "../../lv_conf.h"
2. User_Setup.h → TFT_eSPI/ so TFT_eSPI uses our CYD pin config instead of its
   bundled default (which targets ESP8266 PIN_D7/D5 — wrong for this hardware).
"""
Import("env")
import shutil, os

libdeps = os.path.join(env["PROJECT_DIR"], ".pio", "libdeps", env["PIOENV"])
inc     = os.path.join(env["PROJECT_DIR"], "include")

copies = [
    ("lv_conf.h",   "lvgl"),
    ("User_Setup.h", "TFT_eSPI"),
]

for fname, libname in copies:
    src     = os.path.join(inc, fname)
    lib_dir = os.path.join(libdeps, libname)
    dst     = os.path.join(lib_dir, fname)
    if os.path.isdir(lib_dir):
        shutil.copy2(src, dst)
        print(f"[pre-build] Copied {fname} → {dst}")
    else:
        print(f"[pre-build] {libname} libdeps not found — will copy on next build")
