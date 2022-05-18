@echo off 
set src=%dp0
set dst="out/build/x64-Debug"

robocopy assets %dst%/assets /E
robocopy scenes %dst%/scenes /E
robocopy scene_web %dst%/scene_web /E