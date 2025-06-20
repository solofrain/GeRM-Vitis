import time
import os
from pathlib import Path

# === Config ===

directories_to_watch = [
    "./ZynqDetector/src/common",
    "./ZynqDetector/src/device",
    "./ZynqDetector/src/detector/Germanium"
]

discrete_files = [
    "./ZynqDetector/src/detector_main.cpp",
    "./ZynqDetector/src/UserConfig.cmake"
]

POLL_INTERVAL = 1  # seconds

# === Collect all files ===

def get_all_files( directories_to_watch, discrete_files ):
    from pathlib import Path
    all_files = set()
    for d in directories_to_watch:
        for f in Path(d).rglob("*"):
            if f.is_file():
                all_files.add(str(f.resolve()))
    for f in discrete_files:
        all_files.add(str(Path(f).resolve()))
    return all_files

# === Track last modification times ===

def build_mtime_map(files):
    return {f: os.path.getmtime(f) for f in files if os.path.exists(f)}

# === Your action on change ===

def on_change():
    print( "Rebuilding ZynqDetector..." )
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
            on_change()
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
            on_change()
            last_mtimes = build_mtime_map(watched_files)

except KeyboardInterrupt:
    print("\n🛑 Stopped.")

