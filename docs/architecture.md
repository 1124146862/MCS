# MCS Prototype Architecture

## Adopted Structure

This project now follows the required layered structure:

- `core/`: pure domain data models
- `app/`: document and editor workflow logic
- `ui/`: QML interface and view models
- `infra/`: file paths, importers, and other external-facing services

## Current Mapping

- `main.py`
  - application bootstrap only
- `core/scene`, `core/skeleton`, `core/animation`
  - document-facing prototype models
- `app/document`
  - startup document creation
- `infra/importers`
  - FBX runtime asset preparation and Balsam conversion
- `ui/viewport`, `ui/timeline`, `ui/outliner`, `ui/inspector`
  - modular QML panels
- `ui/viewmodels`
  - Python-to-QML state exposure

## Rules For Future Work

1. Do not put business logic back into `main.py`.
2. Do not let QML directly own file-system or importer logic.
3. New editor actions should go through `app/command`.
4. New tool-state logic should go under `app/tools`.
5. New import/export code should stay under `infra`.
