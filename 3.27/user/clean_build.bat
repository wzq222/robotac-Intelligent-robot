@echo off
echo Cleaning build artifacts in %CD%...
if exist "Listings" rd /s /q "Listings"
if exist "Objects" rd /s /q "Objects"
if exist "project1.axf" del /q "project1.axf"
if exist "project1.lnp" del /q "project1.lnp"
if exist "project1.sct" del /q "project1.sct"
if exist "project1.build_log.htm" del /q "project1.build_log.htm"
if exist "project1.htm" del /q "project1.htm"
echo Done.
pause
