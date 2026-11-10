$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

function Write-Step([string]$Message) {
    Write-Host "`n==========================================================" -ForegroundColor DarkGray
    Write-Host $Message -ForegroundColor Cyan
    Write-Host "==========================================================" -ForegroundColor DarkGray
}

function Refresh-ProcessPath {
    $machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = @($machinePath, $userPath) -join ";"
}

function Get-WinGetCommand {
    $cmd = Get-Command winget -ErrorAction SilentlyContinue
    if (-not $cmd) {
        Write-Host "Windows Package Manager (winget) is not available." -ForegroundColor Red
        Write-Host "Open the official App Installer page and install/update App Installer:" -ForegroundColor Yellow
        Write-Host "https://aka.ms/getwinget" -ForegroundColor Yellow
        Start-Process "https://aka.ms/getwinget"
        throw "winget is required for automatic dependency setup. Install App Installer, reopen this project, and run Run_AutoCore.bat again."
    }
    return $cmd.Source
}

function Install-WinGetPackage([string]$PackageId, [string]$Override = "") {
    $winget = Get-WinGetCommand
    Write-Host "Installing $PackageId ..." -ForegroundColor Yellow
    if ([string]::IsNullOrWhiteSpace($Override)) {
        & $winget install --id $PackageId --exact --source winget --accept-source-agreements --accept-package-agreements --silent
    } else {
        & $winget install --id $PackageId --exact --source winget --accept-source-agreements --accept-package-agreements --override $Override
    }
    if ($LASTEXITCODE -ne 0) {
        throw "winget could not install $PackageId (exit code $LASTEXITCODE). Review the installer output and retry."
    }
    Refresh-ProcessPath
}

function Test-PythonCommand($Command, [string[]]$Prefix = @()) {
    if (-not $Command) { return $false }
    & $Command @Prefix -c "import sys; print(sys.version.split()[0]); raise SystemExit(0 if sys.version_info >= (3,10) else 1)" 2>$null
    return ($LASTEXITCODE -eq 0)
}

function Find-PythonRuntime {
    $pythonCmd = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCmd -and (Test-PythonCommand $pythonCmd.Source @())) {
        return [pscustomobject]@{ Exe = $pythonCmd.Source; Prefix = @() }
    }
    $pyCmd = Get-Command py -ErrorAction SilentlyContinue
    if ($pyCmd -and (Test-PythonCommand $pyCmd.Source @("-3"))) {
        return [pscustomobject]@{ Exe = $pyCmd.Source; Prefix = @("-3") }
    }
    return $null
}

function Test-MsvcBuildTools {
    $programFilesX86 = [Environment]::GetFolderPath("ProgramFilesX86")
    $vswhere = Join-Path $programFilesX86 "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { return $false }
    $installation = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    return (-not [string]::IsNullOrWhiteSpace(($installation | Select-Object -First 1)))
}

try {
    Write-Host "AutoCore v1.6.1 - Automatic Windows Setup, Build, Test and Launch" -ForegroundColor Green
    Write-Host "This setup installs only the missing development tools." -ForegroundColor Gray
    Write-Host "Windows may show an installer/UAC approval prompt. An internet connection is required for first-time setup." -ForegroundColor Yellow

    # 1. Install CMake if it is not already on PATH.
    Write-Step "[1/5] Checking CMake"
    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        Install-WinGetPackage "Kitware.CMake"
    }
    $cmakeCmd = Get-Command cmake -ErrorAction SilentlyContinue
    if (-not $cmakeCmd) {
        $commonCMake = Join-Path $env:ProgramFiles "CMake\bin\cmake.exe"
        if (Test-Path $commonCMake) { $env:Path = (Split-Path $commonCMake) + ";" + $env:Path }
        $cmakeCmd = Get-Command cmake -ErrorAction SilentlyContinue
    }
    if (-not $cmakeCmd) { throw "CMake installation completed but cmake.exe could not be located. Restart Windows and run this launcher again." }
    & $cmakeCmd.Source --version
    if ($LASTEXITCODE -ne 0) { throw "CMake could not run successfully." }

    # 2. Install a C++17 MSVC toolchain if no Visual Studio C++ compiler workload exists.
    Write-Step "[2/5] Checking the Visual C++ compiler"
    if (-not (Test-MsvcBuildTools)) {
        $vsOverride = "--passive --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
        Install-WinGetPackage "Microsoft.VisualStudio.2022.BuildTools" $vsOverride
    }
    if (-not (Test-MsvcBuildTools)) {
        throw "Visual Studio C++ Build Tools are not detected after installation. Reboot if Windows requested it, then run this launcher again."
    }
    Write-Host "Visual Studio C++ Build Tools detected." -ForegroundColor Green

    # 3. Install Python if no Python 3.10+ runtime is available.
    Write-Step "[3/5] Checking Python 3.10+"
    $pythonRuntime = Find-PythonRuntime
    if (-not $pythonRuntime) {
        Install-WinGetPackage "Python.Python.3.14"
        $pythonRuntime = Find-PythonRuntime
    }
    if (-not $pythonRuntime) {
        throw "Python 3.10 or newer was not detected after installation. Restart the terminal/Windows and run this launcher again."
    }
    Write-Host "Python runtime: $($pythonRuntime.Exe)" -ForegroundColor Green

    # 4. Configure in a Windows-specific build folder, compile and test.
    Write-Step "[4/5] Configuring, compiling and testing AutoCore"
    $buildDir = Join-Path $root "build-windows-v1.6.1"
    & $cmakeCmd.Source -S $root -B $buildDir -DCMAKE_BUILD_TYPE=Release
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed." }

    & $cmakeCmd.Source --build $buildDir --config Release --parallel
    if ($LASTEXITCODE -ne 0) { throw "C++ compilation failed." }

    $ctestCmd = Get-Command ctest -ErrorAction SilentlyContinue
    if (-not $ctestCmd) {
        $cmakeBin = Split-Path $cmakeCmd.Source
        $ctestPath = Join-Path $cmakeBin "ctest.exe"
        if (Test-Path $ctestPath) { $ctestExe = $ctestPath } else { throw "CTest was not found with CMake." }
    } else {
        $ctestExe = $ctestCmd.Source
    }
    & $ctestExe --test-dir $buildDir -C Release --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw "AutoCore automated tests failed." }

    $engineCandidates = @(
        (Join-Path $buildDir "Release\autocore_engine.exe"),
        (Join-Path $buildDir "autocore_engine.exe")
    )
    $enginePath = $engineCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $enginePath) { throw "Could not locate autocore_engine.exe after a successful build." }

    # 5. Start the live dashboard. Keep this process in the foreground so Ctrl+C stops both parts.
    Write-Step "[5/5] Starting the live AutoCore dashboard"
    Write-Host "Dashboard address: http://127.0.0.1:8765/" -ForegroundColor Green
    Write-Host "Keep this window open while using AutoCore. Press Ctrl+C to stop." -ForegroundColor Yellow
    $serverArgs = @()
    if ($pythonRuntime.Prefix.Count -gt 0) { $serverArgs += $pythonRuntime.Prefix }
    $serverArgs += @((Join-Path $root "dashboard\server.py"), "--engine", $enginePath, "--open")
    & $pythonRuntime.Exe @serverArgs
    if ($LASTEXITCODE -ne 0) { throw "The AutoCore dashboard server exited with an error." }
}
catch {
    Write-Host "`nAutoCore could not start: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host "If a setup window requested a restart, restart Windows and double-click Run_AutoCore.bat again." -ForegroundColor Yellow
    exit 1
}
