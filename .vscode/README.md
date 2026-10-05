ESP-IDF / Flash helper for this workspace

What I added to make the VS Code "Flash" flow work without manually sourcing the ESP-IDF export script:

- `tools/idf-wrapper` — small executable wrapper that sources your ESP-IDF `export.sh` and then runs `idf.py` with the same arguments. This ensures the correct PATH and python env are active when VS Code runs build/flash commands.
- `.vscode/tasks.json` — two tasks added:
  - "ESP-IDF: Flash (wrapper)" — runs the wrapper and executes `idf.py -p ${config:idf.port} -b ${config:idf.flashBaudRate} flash`.
  - "ESP-IDF: Monitor" — runs `idf.py monitor` via the wrapper.

How to use
- Reload VS Code window (Command Palette: "Developer: Reload Window") so the extension picks up updated `.vscode/settings.json`.
- Use Terminal -> Run Task... and choose "ESP-IDF: Flash (wrapper)" to flash the device. Or run the existing Flash toolbar/button; the wrapper is placed on the extension search path so most extension commands will find it.

Notes
- The wrapper assumes ESP-IDF is at `/home/raymond/esp/esp-idf`. If your IDF is elsewhere, set the environment variable `IDF_PATH_OVERRIDE` before running the wrapper, or edit the wrapper file to point to your location.
- If you prefer the extension to call `idf.py` directly, ensure the extension settings `idf.espIdfPath`, `idf.toolsPath` and `idf.customExtraPaths` are correct and restart VS Code.
