# One local command for the SfM baseline-spacing and plane comparison.
# All recordings and numeric results stay on this computer.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$SourceDir,
    [string]$WorkDir = (Join-Path $env:LOCALAPPDATA 'GrassVIO-SfM'),
    [switch]$UseSourceInPlace,
    [switch]$SkipInstall,
    [switch]$OnlySpacing
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$branch = 'codex/multiview-and-inertial-validation'
$repository = 'https://github.com/hvsr-uw/grassvio-tools.git'
$checkout = Join-Path $WorkDir 'tools-sfm'
$source = (Resolve-Path -LiteralPath $SourceDir).Path
$inputDir = if ($UseSourceInPlace) { $source } else { Join-Path $WorkDir 'recordings' }
$results = Join-Path $WorkDir 'results'
$recovery = Join-Path $results 'recovery'
$multiview = Join-Path $results 'multiview'
$comparison = Join-Path $results 'sfm-spacing'
$null = New-Item -ItemType Directory -Force -Path $WorkDir, $results, $comparison

$selected = @(Get-ChildItem -LiteralPath $source -Filter '*.db3' -File |
    Where-Object { $_.Name -match '^20261119_\d{6}\.db3$' } |
    Sort-Object Name)
if ($selected.Count -eq 0) {
    throw 'No November 19 .db3 files in the source folder.'
}
Write-Host "Using $($selected.Count) recording(s); files are never uploaded."

if (-not $UseSourceInPlace) {
    $null = New-Item -ItemType Directory -Force -Path $inputDir
    foreach ($file in $selected) {
        $destination = Join-Path $inputDir $file.Name
        if ((Test-Path -LiteralPath $destination) -and
            (Get-Item -LiteralPath $destination).Length -eq $file.Length) {
            Write-Host "Already copied: $($file.Name)"
            continue
        }
        $partial = "$destination.partial"
        if (Test-Path -LiteralPath $partial) {
            Remove-Item -LiteralPath $partial -Force
        }
        Copy-Item -LiteralPath $file.FullName -Destination $partial
        if ((Get-Item -LiteralPath $partial).Length -ne $file.Length) {
            throw "Incomplete recording copy: $($file.Name)"
        }
        Move-Item -LiteralPath $partial -Destination $destination -Force
    }
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw 'Git for Windows is required.'
}
if (-not (Test-Path -LiteralPath (Join-Path $checkout '.git'))) {
    & git clone --single-branch --branch $branch $repository $checkout
    if ($LASTEXITCODE -ne 0) { throw 'Could not clone SfM review branch.' }
} else {
    $dirty = @(& git -C $checkout status --porcelain)
    if ($LASTEXITCODE -ne 0 -or $dirty.Count -gt 0) {
        throw 'The SfM checkout has uncommitted changes; it was not updated.'
    }
    $actual = (& git -C $checkout branch --show-current).Trim()
    if ($actual -ne $branch) {
        throw "Expected branch $branch but found $actual."
    }
    & git -C $checkout pull --ff-only origin $branch
    if ($LASTEXITCODE -ne 0) { throw 'Could not fast-forward SfM branch.' }
}

$pythonBase = $null
if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -c 'import sys; assert sys.version_info >= (3,9)' | Out-Null
    if ($LASTEXITCODE -eq 0) { $pythonBase = 'py' }
}
if (-not $pythonBase -and (Get-Command python -ErrorAction SilentlyContinue)) {
    & python -c 'import sys; assert sys.version_info >= (3,9)' | Out-Null
    if ($LASTEXITCODE -eq 0) { $pythonBase = 'python' }
}
if (-not $pythonBase) { throw 'Python 3.9 or newer is required.' }

$venv = Join-Path $checkout '.venv'
$python = Join-Path $venv 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $python)) {
    if ($pythonBase -eq 'py') { & py -3 -m venv $venv }
    else { & python -m venv $venv }
    if ($LASTEXITCODE -ne 0) { throw 'Could not create Python environment.' }
}
if (-not $SkipInstall) {
    & $python -m pip install -r (Join-Path $checkout 'requirements-analysis.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
}

$names = @($selected | ForEach-Object { $_.Name })

function Invoke-Stage {
    param([string]$ScriptName, [string[]]$ScriptArgs)
    Write-Host "Running $ScriptName"
    & $python (Join-Path $checkout $ScriptName) @ScriptArgs
    if ($LASTEXITCODE -ne 0) { throw "$ScriptName failed; inspect results for logs." }
}

if (-not $OnlySpacing) {
    $recoveryArgs = @(
        '--input-dir', $inputDir,
        '--output-dir', $recovery,
        '--names'
    ) + $names
    Invoke-Stage -ScriptName 'scripts/run_motion_recovery.py' -ScriptArgs $recoveryArgs

    $multiviewArgs = @(
        '--input-dir', $inputDir,
        '--recovery-dir', $recovery,
        '--output-dir', $multiview,
        '--adaptive-gaps', '--names'
    ) + $names
    Invoke-Stage -ScriptName 'scripts/run_multiview_validation.py' -ScriptArgs $multiviewArgs
}

$spacingArgs = @(
    '--input-dir', $inputDir,
    '--multiview-dir', $multiview,
    '--output-dir', $comparison,
    '--gaps', '5', '25', '50',
    '--anchor-step', '50',
    '--names'
) + $names
Invoke-Stage -ScriptName 'scripts/run_sfm_spacing_plane_check.py' -ScriptArgs $spacingArgs

Invoke-Stage 'scripts/run_sfm_spacing_synthetic_controls.py' @(
    '--out', (Join-Path $comparison 'synthetic.json')
)

Write-Host "Finished SfM spacing comparison."
Write-Host "Summary: $(Join-Path $comparison 'summary.csv')"
Write-Host "Methods and results: $(Join-Path $comparison 'report.md')"
Write-Host 'Raw recordings remain in local storage; no files were uploaded.'
