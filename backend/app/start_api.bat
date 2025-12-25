@echo off
echo Starting Physical AI and Robotics Learning Framework API...
echo.

REM Change to the app directory
cd /d "%~dp0"

REM Activate virtual environment if it exists
if exist ..\venv\Scripts\activate.bat (
    call ..\venv\Scripts\activate.bat
    echo Virtual environment activated.
    echo.
)

REM Install dependencies if not already installed
echo Installing dependencies...
pip install -r requirements.txt
echo.

REM Start the FastAPI server
echo Starting API server on http://localhost:8000...
uvicorn api:app --host 0.0.0.0 --port 8000 --reload

pause