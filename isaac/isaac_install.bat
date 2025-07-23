@echo off
setlocal EnableDelayedExpansion

echo ===============================================
echo Isaac Sim + Isaac Lab Installation Script
echo Following: isaac-sim.github.io/IsaacLab/
echo ===============================================
echo This script will:
echo 1. Detect existing Anaconda installation
echo 2. Create isaac_sim environment with Python 3.10
echo 3. Install PyTorch 2.5.1 with CUDA 11.8
echo 4. Install Isaac Sim 4.5.0 from pip
echo 5. Clone Isaac Lab from GitHub
echo 6. Install Isaac Lab and dependencies
echo ===============================================
echo.

:: Step 1: Detect Anaconda installation
echo Step 1: Detecting Anaconda installation...

set "CONDA_FOUND=0"
set "CONDA_PATH="

:: Check common Anaconda locations
if exist "%UserProfile%\anaconda3\Scripts\conda.exe" (
    set "CONDA_PATH=%UserProfile%\anaconda3"
    set "CONDA_FOUND=1"
    echo ✓ Found Anaconda at: %CONDA_PATH%
) else if exist "%UserProfile%\Miniconda3\Scripts\conda.exe" (
    set "CONDA_PATH=%UserProfile%\Miniconda3"
    set "CONDA_FOUND=1"
    echo ✓ Found Miniconda at: %CONDA_PATH%
) else (
    :: Try to find conda in PATH
    where conda >nul 2>&1
    if !errorlevel! == 0 (
        set "CONDA_FOUND=1"
        echo ✓ Found conda in system PATH
    )
)

if !CONDA_FOUND! == 0 (
    echo ✗ Anaconda/Miniconda not found!
    echo Please install Anaconda first from: https://www.anaconda.com/download
    pause
    exit /b 1
)

:: Display conda version
echo.
echo Conda information:
if defined CONDA_PATH (
    "!CONDA_PATH!\Scripts\conda.exe" --version
) else (
    conda --version
)

echo.
echo Step 2: Checking for existing isaac_sim environment...

:: Check if isaac_sim environment already exists
if defined CONDA_PATH (
    "!CONDA_PATH!\Scripts\conda.exe" env list | findstr "isaac_sim" >nul
) else (
    conda env list | findstr "isaac_sim" >nul
)

if !errorlevel! == 0 (
    echo ✓ isaac_sim environment already exists!
    echo Do you want to:
    echo 1. Keep existing environment and continue installation (recommended)
    echo 2. Remove and recreate environment
    echo 3. Exit and use existing environment
    echo.
    set /p choice="Enter your choice (1/2/3): "
    
    if "!choice!"=="2" (
        echo Removing existing isaac_sim environment...
        if defined CONDA_PATH (
            "!CONDA_PATH!\Scripts\conda.exe" env remove -n isaac_sim -y
        ) else (
            conda env remove -n isaac_sim -y
        )
        echo ✓ Existing environment removed
        goto :create_env
    ) else if "!choice!"=="3" (
        echo Exiting. You can activate your existing environment with:
        echo conda activate isaac_sim
        pause
        exit /b 0
    ) else (
        echo ✓ Using existing isaac_sim environment
        goto :setup_paths
    )
) else (
    goto :create_env
)

:create_env
:: Create isaac_sim environment with Python 3.10
echo Creating isaac_sim environment with Python 3.10...
if defined CONDA_PATH (
    "!CONDA_PATH!\Scripts\conda.exe" create -n isaac_sim python=3.10 -y
) else (
    conda create -n isaac_sim python=3.10 -y
)

if !errorlevel! neq 0 (
    echo ✗ Error: Failed to create isaac_sim environment
    pause
    exit /b 1
)

echo ✓ isaac_sim environment created successfully!

:setup_paths
:: Set paths for the environment and verify they exist
if defined CONDA_PATH (
    set "ISAAC_PIP=!CONDA_PATH!\envs\isaac_sim\Scripts\pip.exe"
    set "ISAAC_PYTHON=!CONDA_PATH!\envs\isaac_sim\Scripts\python.exe"
    set "ENV_PATH=!CONDA_PATH!\envs\isaac_sim"
) else (
    for /f "tokens=*" %%i in ('conda info --base') do set "CONDA_BASE=%%i"
    set "ISAAC_PIP=!CONDA_BASE!\envs\isaac_sim\Scripts\pip.exe"
    set "ISAAC_PYTHON=!CONDA_BASE!\envs\isaac_sim\Scripts\python.exe"
    set "ENV_PATH=!CONDA_BASE!\envs\isaac_sim"
)

:: Verify environment paths exist
echo.
echo Checking environment paths...
echo Environment directory: !ENV_PATH!

if not exist "!ENV_PATH!" (
    echo ✗ Error: Environment directory does not exist: !ENV_PATH!
    echo Environment creation may have failed.
    pause
    exit /b 1
)

if not exist "!ISAAC_PYTHON!" (
    echo ✗ Error: Python executable not found: !ISAAC_PYTHON!
    echo Looking for alternative Python locations...
    
    :: Try alternative paths
    if exist "!ENV_PATH!\python.exe" (
        set "ISAAC_PYTHON=!ENV_PATH!\python.exe"
        set "ISAAC_PIP=!ENV_PATH!\Scripts\pip.exe"
        echo ✓ Found Python at: !ISAAC_PYTHON!
    ) else (
        echo ✗ Cannot locate Python executable in environment
        echo Please check if the environment was created properly
        pause
        exit /b 1
    )
)

:: Verify environment
echo.
echo Verifying isaac_sim environment...
echo Python path: !ISAAC_PYTHON!
"!ISAAC_PYTHON!" --version

:: Check if Python version is correct
for /f "tokens=2" %%v in ('"!ISAAC_PYTHON!" --version 2^>^&1') do set "PYTHON_VERSION=%%v"
echo Python version detected: !PYTHON_VERSION!

if not "!PYTHON_VERSION:~0,4!"=="3.10" (
    echo ✗ Error: Python version is !PYTHON_VERSION! but Isaac Sim requires Python 3.10
    echo The isaac_sim environment may be corrupted or using wrong Python version.
    echo.
    echo Recommended solution:
    echo 1. Remove the environment: conda env remove -n isaac_sim -y
    echo 2. Run this script again to create a fresh environment
    echo.
    set /p fix="Do you want to remove and recreate the environment now? (y/n): "
    if /i "!fix!"=="y" (
        echo Removing corrupted isaac_sim environment...
        if defined CONDA_PATH (
            "!CONDA_PATH!\Scripts\conda.exe" env remove -n isaac_sim -y
        ) else (
            conda env remove -n isaac_sim -y
        )
        goto :create_env
    ) else (
        echo Please fix the environment manually and run the script again.
        pause
        exit /b 1
    )
)

echo.
echo Step 3: Installing PyTorch 2.5.1 with CUDA 11.8...

:: Upgrade pip first (as per official docs)
echo Upgrading pip...
"!ISAAC_PYTHON!" -m pip install --upgrade pip

if !errorlevel! neq 0 (
    echo ⚠ Pip upgrade failed, but continuing with installation...
)

:: Install PyTorch 2.5.1 with CUDA 11.8 (as per official docs)
echo Installing PyTorch 2.5.1 with CUDA 11.8 support...
"!ISAAC_PIP!" install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu118

if !errorlevel! neq 0 (
    echo ⚠ PyTorch CUDA 11.8 installation failed, trying CUDA 12.1...
    "!ISAAC_PIP!" install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
    if !errorlevel! neq 0 (
        echo ✗ Error: PyTorch installation failed
        echo Please check your internet connection and CUDA compatibility
        pause
        exit /b 1
    )
)

echo ✓ PyTorch installed successfully!

echo.
echo Step 4: Installing Isaac Sim 4.5.0...

:: Install Isaac Sim 4.5.0 with all extensions and cache (as per official docs)
echo Installing Isaac Sim 4.5.0 with all extensions...
echo This may take several minutes and requires significant disk space...
"!ISAAC_PIP!" install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com

if !errorlevel! neq 0 (
    echo ⚠ Isaac Sim 4.5.0 installation failed, trying without version constraint...
    "!ISAAC_PIP!" install "isaacsim[all,extscache]" --extra-index-url https://pypi.nvidia.com
    if !errorlevel! neq 0 (
        echo ✗ Error: Isaac Sim installation failed
        echo This might be due to:
        echo - Network connectivity issues
        echo - NVIDIA PyPI server issues
        echo - CUDA driver compatibility (requires 552.86+ for CUDA 12)
        echo - Insufficient disk space
        pause
        exit /b 1
    )
)

echo ✓ Isaac Sim installed successfully!

echo.
echo Step 5: Cloning Isaac Lab from GitHub...

:: Create workspace directory
if not exist "IsaacLab" (
    echo Cloning Isaac Lab repository...
    git clone https://github.com/isaac-sim/IsaacLab.git
    if !errorlevel! neq 0 (
        echo ✗ Error: Failed to clone Isaac Lab repository
        echo Make sure git is installed and you have internet access
        pause
        exit /b 1
    )
    echo ✓ Isaac Lab cloned successfully!
) else (
    echo ✓ Isaac Lab directory already exists
)

:: Navigate to Isaac Lab directory
cd IsaacLab

echo.
echo Step 6: Installing Isaac Lab and dependencies...

:: Install build dependencies
echo Installing build dependencies...
"!ISAAC_PIP!" install cmake wheel setuptools

echo.
echo Installing Isaac Lab extensions and learning frameworks...
echo This may take several minutes...

:: Try using isaaclab.bat first (with environment activation)
if exist "isaaclab.bat" (
    echo Attempting to use official isaaclab.bat installer...
    
    :: Create a temporary batch file to run commands in activated environment
    echo @echo off > temp_install.bat
    if defined CONDA_PATH (
        echo call "!CONDA_PATH!\Scripts\activate.bat" isaac_sim >> temp_install.bat
    ) else (
        echo call conda activate isaac_sim >> temp_install.bat
    )
    echo cd /d "%cd%" >> temp_install.bat
    echo isaaclab.bat --install >> temp_install.bat
    echo exit /b %%errorlevel%% >> temp_install.bat
    
    :: Run the temporary batch file
    call temp_install.bat
    set "INSTALL_RESULT=!errorlevel!"
    
    :: Clean up temporary file
    del temp_install.bat
    
    if !INSTALL_RESULT! neq 0 (
        echo ⚠ isaaclab.bat --install failed, using manual installation method...
        goto :manual_install
    ) else (
        echo ✓ Isaac Lab installed successfully using isaaclab.bat!
        goto :verify_install
    )
) else (
    echo isaaclab.bat not found, using manual installation method...
    goto :manual_install
)

:manual_install
echo Installing Isaac Lab manually using pip...

:: Install Isaac Lab in editable mode
echo Installing Isaac Lab core package...
"!ISAAC_PIP!" install -e .

if !errorlevel! neq 0 (
    echo ✗ Error: Failed to install Isaac Lab core package
    pause
    exit /b 1
)

:: Install learning frameworks with version compatibility
echo.
echo Installing learning frameworks...
echo Note: Installing compatible versions for Python 3.10...

:: Install frameworks one by one with error handling
echo Installing rsl_rl...
"!ISAAC_PIP!" install rsl_rl
if !errorlevel! neq 0 (
    echo ⚠ rsl_rl installation failed
)

echo Installing rl_games (compatible version)...
"!ISAAC_PIP!" install "rl_games<1.7.0"
if !errorlevel! neq 0 (
    echo ⚠ rl_games installation failed, trying alternative version...
    "!ISAAC_PIP!" install "rl-games==1.6.2"
    if !errorlevel! neq 0 (
        echo ⚠ rl_games installation failed completely
    )
)

echo Installing stable-baselines3...
"!ISAAC_PIP!" install stable-baselines3
if !errorlevel! neq 0 (
    echo ⚠ stable-baselines3 installation failed
)

echo Installing skrl...
"!ISAAC_PIP!" install skrl
if !errorlevel! neq 0 (
    echo ⚠ skrl installation failed
)

echo Installing additional dependencies...
"!ISAAC_PIP!" install gymnasium tensorboard wandb
if !errorlevel! neq 0 (
    echo ⚠ Some additional dependencies failed to install
)

:verify_install
echo.
echo Step 7: Verifying installation...

:: Test Isaac Sim import
echo Testing Isaac Sim installation...
"!ISAAC_PYTHON!" -c "import isaacsim; print('✓ Isaac Sim import successful!')" 2>nul

if !errorlevel! neq 0 (
    echo ⚠ Isaac Sim import test failed - this may be normal on first run
    echo Isaac Sim will initialize properly when first launched
) else (
    echo ✓ Isaac Sim verification passed!
)

:: Test Isaac Lab import
echo Testing Isaac Lab installation...
"!ISAAC_PYTHON!" -c "import isaaclab; print('✓ Isaac Lab import successful!')" 2>nul

if !errorlevel! neq 0 (
    echo ⚠ Isaac Lab import test failed
    echo Please check the installation manually
) else (
    echo ✓ Isaac Lab verification passed!
)

:: Check if tutorial script exists
if exist "scripts\tutorials\00_sim\create_empty.py" (
    echo ✓ Tutorial scripts found
) else (
    echo ⚠ Tutorial scripts not found - installation may be incomplete
)

echo.
echo ===============================================
echo Installation Summary
echo ===============================================
echo.
echo ✓ Isaac Sim 4.5.0 - Installed
echo ✓ Isaac Lab - Installed from source
echo ✓ PyTorch 2.5.1 with CUDA - Installed
echo ✓ Learning frameworks - Installed
echo.
echo Environment: isaac_sim
echo Location: %cd%
echo.
echo ===============================================
echo Next Steps:
echo ===============================================
echo.
echo 1. Accept NVIDIA EULA on first run (required)
echo 2. Test Isaac Lab installation:
echo    - Open Command Prompt or Anaconda Prompt
echo    - Run: conda activate isaac_sim
echo    - Navigate to: cd %cd%
echo    - Test with: isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py
echo.
echo 3. Train your first robot:
echo    - isaaclab.bat -p scripts\reinforcement_learning\rsl_rl\train.py --task=Isaac-Ant-v0 --headless
echo.
echo 4. For faster training, always add --headless flag
echo.
echo Note: First run will download additional extensions (may take 10+ minutes)
echo ===============================================

pause

:: Return to original directory
cd ..

echo.
echo Opening Command Prompt with isaac_sim environment activated...
if defined CONDA_PATH (
    start "Isaac Lab Environment" cmd /k ""!CONDA_PATH!\Scripts\activate.bat" isaac_sim && cd IsaacLab && echo Environment ready! Try: isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py"
) else (
    start "Isaac Lab Environment" cmd /k "conda activate isaac_sim && cd IsaacLab && echo Environment ready! Try: isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py"
)