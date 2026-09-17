@echo off
rem Bring the OCP CAD Viewer up on a printed part and keep it current.
rem
rem   watch.cmd                  the rail mount assembly, redrawn every time you save
rem   watch.cmd --section -23    sliced at that station, which is the only way to
rem                              see what the jaws are doing -- from outside, the
rem                              plate hides the whole clamp
rem   watch.cmd --list           what can be shown
rem   watch.cmd --browser        a standalone viewer in a browser tab, instead of
rem                              the OCP CAD Viewer panel in VS Code
rem
rem Everything you pass goes through to the part's own script, so whatever
rem `python cad\oak_rail_mount.py --show ...` draws, this draws and then keeps
rem current. The work is all in cad\watch.py, which redraws on every save.
rem
rem WHAT THIS ADDS IS THE INTERPRETER. build123d is deliberately not in
rem requirements.txt -- the rover never needs it, nothing in cad\ is deployed --
rem so the Python that runs the rover is not the Python that can draw a part.
rem This looks for one that can: %CAD_PYTHON% if you set it, else the repository's
rem .venv, else whatever `python` is on PATH. If none of them has build123d it
rem says so and stops, rather than handing you an ImportError from a subprocess.
rem
rem IT DRAWS INTO THE VS CODE PANEL by default, because a part beside the code it
rem is drawn from is the point of watching it at all. Open it with ctrl-shift-P,
rem "OCP CAD Viewer: Open viewer" -- in either order, because until a viewer
rem exists this waits for one rather than building a part it has nowhere to put.
rem
rem WHICH PORT THE PANEL IS ON IS NOT ASSUMED. 3939 is only where the extension
rem starts looking for a free one, so a standalone viewer or a second window puts
rem the panel on 3940 and up. Every viewer registers itself in ~\.ocpvscode and
rem cad\watch.py reads that, so it finds the panel wherever it landed.
rem
rem AND IT PROVES NOTHING ABOUT THE FIT. `python cad\oak_rail_mount.py` is what
rem checks the part against a model of the rail and exits non-zero when it does
rem not fit; this is for looking at it while you work on it.
setlocal
pushd "%~dp0"

set "PY=%CAD_PYTHON%"
if not defined PY if exist ".venv\Scripts\python.exe" set "PY=%CD%\.venv\Scripts\python.exe"
if not defined PY set "PY=python"

"%PY%" -c "import sys; sys.exit(0)" >nul 2>nul
if errorlevel 1 (
    echo No Python found. Set CAD_PYTHON to one that has build123d in it,
    echo or make the scratch environment cad\README.md describes.
    popd
    exit /b 1
)

rem find_spec rather than import, because importing build123d costs seconds and
rem this runs on every start.
"%PY%" -c "import importlib.util,sys; sys.exit(0 if importlib.util.find_spec('build123d') and importlib.util.find_spec('ocp_vscode') else 1)" >nul 2>nul
if errorlevel 1 (
    echo %PY% has no build123d, or no ocp_vscode. Nothing in cad\ is deployed and
    echo the rover never needs either, so they live in a scratch environment:
    echo.
    echo     python -m venv .venv
    echo     .venv\Scripts\pip install build123d ocp_vscode
    echo.
    echo Or set CAD_PYTHON to an interpreter that already has them.
    popd
    exit /b 1
)

"%PY%" cad\watch.py %*
set WATCH_EXIT=%ERRORLEVEL%
popd
exit /b %WATCH_EXIT%
