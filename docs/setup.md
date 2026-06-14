# MCS Python Prototype Setup

## Project Root

- Active Python Qt prototype root: `D:\创业\MCS-PY\MCS`

## Current Stack

- Frontend: `QML / Qt Quick / Qt Quick 3D`
- Backend: `Python / PySide6`
- Python kit: `Python 3.12 (64-bit)`
- Virtual environment: `D:\创业\MCS-PY\MCS\.qtcreator\Python_3_12_64_bit_venv`
- IDE: `Qt Creator`

## Current Prototype State

The current V0.5 prototype already includes:

- a dark 3D viewport
- a central preview area
- a ground plane plus fake editor grid overlay
- a default camera and directional light
- orbit rotation and wheel zoom
- a right-side fake control panel
- a bottom fake timeline area

## Default FBX Loading Rule

- On startup, the app looks for `assets/source/model.fbx`.
- If that file is missing, it tries to move `C:\Users\11241\Desktop\model.fbx` into the project.
- If a Desktop copy and a project copy both exist and the files are identical, the Desktop duplicate is removed automatically.
- The app prefers to show the generated runtime model from `assets/generated/Model.qml`.
- If runtime import is unavailable, the viewport falls back to a built-in placeholder body.

## Working Convention

- Treat `D:\创业\MCS-PY\MCS` as the active Python prototype workspace.
- Put project-specific notes under `docs/`.
- Use Python first for algorithm and data-flow experiments.
- Keep the first prototype focused on viewport interaction before adding mocap data streams, skeleton solving, retargeting, timeline editing, or hardware integration.
- Organize all future code using the layered structure in `core/`, `app/`, `ui/`, and `infra/`.
