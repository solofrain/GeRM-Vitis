import os
import time
from pathlib import Path

import vitis

pwd = os.getcwd()

client = vitis.create_client()
client.set_workspace(path=pwd)

app = client.get_component(name="ZynqDetector")


# === Config ===

directories_to_watch = [
    "./ZynqDetector/src/common",
    "./ZynqDetector/src/device",
    "./ZynqDetector/src/detector/Germanium"
]

discrete_files = [
	"./ZynqDetector/src/app.yaml",
	"./ZynqDetector/src/CMakeLists.txt",
	"./ZynqDetector/src/compile_commands.json",
	"./ZynqDetector/src/lscript.ld",
    "./ZynqDetector/src/UserConfig.cmake",
    "./ZynqDetector/src/detector_main.cpp"
]

POLL_INTERVAL = 1  # seconds

# === Collect all files ===

def get_all_files( directories_to_watch, discrete_files ):
    from pathlib import Path

    allowed_extensions = {".cpp", ".hpp", ".tpp"}
    all_files = set()

    for d in directories_to_watch:
        for f in Path(d).rglob("*"):
            if f.is_file() and f.suffix in allowed_extensions:
                all_files.add(str(f.resolve()))

    for f in discrete_files:
        all_files.add(str(Path(f).resolve()))

    return all_files

# === Track last modification times ===

def build_mtime_map(files):
    return {f: os.path.getmtime(f) for f in files if os.path.exists(f)}

# === Your action on change ===

def on_change( app ):
    
    from datetime import datetime

    print( "\n\n\n\n\n\n\n\n" )
    print("\033c", end="")
    print( "========================================================================" )
    print( f"==  ZynqDetector source file(s) changed  @ {datetime.now()} ==" )
    print( "==                          Rebuilding...                             ==" )
    print( "========================================================================" )
    #app.clean()  # enable if platform updated
    app.build()

# === Poll loop ===

watched_files = get_all_files( directories_to_watch, discrete_files )
last_mtimes = build_mtime_map(watched_files)

print("👀 Polling for file changes... Ctrl+C to stop.")
try:
    while True:
        time.sleep(POLL_INTERVAL)
        current_files = get_all_files( directories_to_watch, discrete_files )

        # Check for added or removed files
        if current_files != watched_files:
            print("📁 File added or removed.")
            watched_files = current_files
            last_mtimes = build_mtime_map(watched_files)
            on_change( app )
            continue

        # Check for modified files
        changed = False
        for f in watched_files:
            if not os.path.exists(f):
                continue
            mtime = os.path.getmtime(f)
            if f not in last_mtimes or mtime != last_mtimes[f]:
                print(f"📄 Modified: {f}")
                changed = True
                break

        if changed:
            on_change( app )
            last_mtimes = build_mtime_map(watched_files)

except KeyboardInterrupt:
    print("\n🛑 Stopped.")

