# Run from Windows PowerShell 5.1+ after signing in to Box Drive or
# downloading the original recordings into a local folder.
#
# Examples:
# powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_local_windows.ps1 -SourceDir "$HOME\Downloads\CS566"
# powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_local_windows.ps1 -SourceDir "C:\Users\me\Box\GrassVIO" -MaxFrames 300
#
# Only copies authorized recordings from SourceDir. Nothing is uploaded.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$SourceDir,

    [string]$WorkDir = (Join-Path $env:LOCALAPPDATA 'GrassVIO'),

    # Six November 19 recordings and the older 080033 comparison.
    [string]$NamePattern = '^(20261119_\d{6}|20261007_080033)\.db3$',

    # Process a manageable subset first. Use -MaxFrames 0 only after
    # checking resources for a full-length run.
    [ValidateRange(0, 1000000)]
    [int]$MaxFrames = 300,

    [switch]$SkipCopy,
    [switch]$UseSourceInPlace,
    [switch]$SkipInstall,
    [string]$SummaryMirrorDir = '',
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$branch = 'codex/nov20-recording-validation-runner'
$repoUrl = 'https://github.com/hvsr-uw/grassvio-tools.git'
$repo = Join-Path $WorkDir 'tools'
$source = (Resolve-Path -LiteralPath $SourceDir -ErrorAction Stop).Path
$inputDir = if ($UseSourceInPlace) { $source } else { Join-Path $WorkDir 'recordings' }
$outputDir = Join-Path $WorkDir 'results'
$null = New-Item -ItemType Directory -Force -Path $WorkDir, $outputDir
if (-not $UseSourceInPlace) {
    $null = New-Item -ItemType Directory -Force -Path $inputDir
}

Write-Host "GrassVIO local workspace: $WorkDir"
Write-Host "Source: $source"
Write-Host "Data stays on this computer. No Box or GitHub uploads."

if (-not $SkipCopy -and -not $UseSourceInPlace) {
    $selected = @(Get-ChildItem -LiteralPath $source -File -Recurse -Filter '*.db3' |
                  Where-Object { $_.Name -match $NamePattern })
    if ($selected.Count -eq 0) {
        throw "No matching .db3 recordings found in $source. Adjust -SourceDir or -NamePattern."
    }
    $selected = @($selected | Sort-Object FullName)
    $duplicateNames = @($selected | Group-Object Name | Where-Object { $_.Count -gt 1 })
    if ($duplicateNames.Count -gt 0) {
        throw "Multiple recordings have the same filename: $($duplicateNames.Name -join ', '). Use a narrower source folder."
    }
    Write-Host "Matched $($selected.Count) recording(s):"
    foreach ($file in $selected) {
        Write-Host ("  {0} ({1:N2} GB)" -f $file.Name, ($file.Length / 1GB))
    }
    if ($DryRun) {
        Write-Host 'Dry run: nothing copied or executed.'
        exit 0
    }
    foreach ($file in $selected) {
        $destination = Join-Path $inputDir $file.Name
        if ((Test-Path -LiteralPath $destination) -and
            (Get-Item -LiteralPath $destination).Length -eq $file.Length) {
            Write-Host "Already copied: $($file.Name)"
            continue
        }
        $temporary = $destination + '.partial'
        if (Test-Path -LiteralPath $temporary) {
            Remove-Item -LiteralPath $temporary -Force
        }
        Write-Host "Copying $($file.Name) into private local storage..."
        Copy-Item -LiteralPath $file.FullName -Destination $temporary -ErrorAction Stop
        if ((Get-Item -LiteralPath $temporary).Length -ne $file.Length) {
            throw "Incomplete copy of $($file.Name). Check the Box download status and retry."
        }
        if (Test-Path -LiteralPath $destination) {
            Remove-Item -LiteralPath $destination -Force
        }
        Move-Item -LiteralPath $temporary -Destination $destination
    }
} elseif ($DryRun) {
    Write-Host 'Dry run: copy skipped; nothing executed.'
    exit 0
}

if ($UseSourceInPlace) {
    Write-Host 'Using the source folder in place. Ensure Box Drive files are available offline.'
}
$recordings = @(Get-ChildItem -LiteralPath $inputDir -File -Filter '*.db3' |
                Where-Object { $_.Name -match $NamePattern })
if ($recordings.Count -eq 0) {
    throw "No local recordings available in $inputDir."
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw 'Git is not installed. Install Git for Windows and reopen PowerShell.'
}
if (-not (Test-Path -LiteralPath (Join-Path $repo '.git'))) {
    Write-Host 'Cloning the review branch...'
    & git clone --single-branch --branch $branch $repoUrl $repo
    if ($LASTEXITCODE -ne 0) { throw 'Git clone failed.' }
} else {
    if (-not (Test-Path -LiteralPath (Join-Path $repo 'scripts\run_recording_batch.py'))) {
        throw "Existing folder does not contain the expected GrassVIO checkout: $repo"
    }
    $status = & git -C $repo status --porcelain
    if ($LASTEXITCODE -ne 0 -or $status) {
        throw 'The local checkout has changes. Preserve them before updating the review branch.'
    }
    $currentBranch = (& git -C $repo branch --show-current).Trim()
    if ($currentBranch -ne $branch) {
        throw "Local checkout is on $currentBranch, expected $branch. I did not change it."
    }
    Write-Host 'Fast-forwarding the local review branch...'
    & git -C $repo pull --ff-only origin $branch
    if ($LASTEXITCODE -ne 0) { throw 'Git update failed without changing history.' }
}

$pythonBase = $null
if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -c 'import sys; assert sys.version_info >= (3,9)' | Out-Null
    if ($LASTEXITCODE -eq 0) { $pythonBase = 'py' }
}
if (-not $pythonBase -and (Get-Command python -ErrorAction SilentlyContinue)) {
    & python -c 'import sys; assert sys.version_info >= (3,9)' 2>$null
    if ($LASTEXITCODE -eq 0) { $pythonBase = 'python' }
}
if (-not $pythonBase) {
    throw 'Python 3.9+ is needed. Install Python for Windows, then rerun.'
}

$venv = Join-Path $repo '.venv'
$python = Join-Path $venv 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) {
    Write-Host 'Creating the local Python environment...'
    if ($pythonBase -eq 'py') {
        & py -3 -m venv $venv
    } else {
        & python -m venv $venv
    }
    if ($LASTEXITCODE -ne 0) { throw 'Could not create Python environment.' }
}
if (-not $SkipInstall) {
    Write-Host 'Installing or checking Python dependencies...'
    & $python -m pip install -r (Join-Path $repo 'requirements-dev.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Python dependency installation failed.' }
}

Write-Host 'Validating the copied SQLite recordings...'
# A lightweight integrity check before expensive image decoding. This
# does not assert the original recording's calibration or motion is correct.
$check = @'
from pathlib import Path
import sqlite3
import sys
for filename in sys.argv[1:]:
    p=Path(filename)
    with sqlite3.connect(p.resolve().as_uri() + "?mode=ro", uri=True) as con:
        names={row[0] for row in con.execute("select name from sqlite_master where type='table'")}
        if not {"topics", "messages"}.issubset(names):
            raise ValueError(f"{p.name}: missing ROS 2 tables")
        color=con.execute("select count(*) from topics where type='sensor_msgs/msg/Image'").fetchone()[0]
        imu=con.execute("select count(*) from topics where type='sensor_msgs/msg/Imu'").fetchone()[0]
        if color < 1 or imu < 1:
            raise ValueError(f"{p.name}: missing image or IMU topics")
        print(f"{p.name}: {color} image, {imu} IMU topics", flush=True)
'@
$names = @($recordings | Sort-Object Name | ForEach-Object { $_.FullName })
& $python -c $check @names
if ($LASTEXITCODE -ne 0) { throw 'Recording validation failed.' }

Write-Host "Running image-motion comparisons with MaxFrames=$MaxFrames ..."
$runArgs = @(
    (Join-Path $repo 'scripts\run_recording_batch.py'),
    '--input-dir', $inputDir,
    '--output-dir', $outputDir,
    '--max-frames', "$MaxFrames",
    '--window', '25',
    '--stride', '5',
    '--blend', 'best',
    '--debug-images'
)
& $python @runArgs
if ($LASTEXITCODE -ne 0) {
    Write-Warning "One or more recordings failed. Check the logs in $outputDir."
    exit $LASTEXITCODE
}

Write-Host ""
if (-not [string]::IsNullOrWhiteSpace($SummaryMirrorDir)) {
    $null = New-Item -ItemType Directory -Force -Path $SummaryMirrorDir
    foreach ($filename in @('summary.csv', 'summary.json')) {
        Copy-Item -LiteralPath (Join-Path $outputDir $filename) -Destination (Join-Path $SummaryMirrorDir $filename) -Force
    }
    Write-Host "Copied only summary CSV/JSON to: $SummaryMirrorDir"
    Write-Host 'If that folder syncs to a cloud service, these summaries will sync there.'
}
Write-Host "Finished. Compare results at: $outputDir"
Write-Host ("Summary: " + (Join-Path $outputDir 'summary.csv'))
Write-Host 'These are image-motion checks, not proof of correct IMU or SfM translation.'
