# Apply .clang-format to the plugin's own sources.
#
# ARCHITECTURE.md says formatting is enforced by .clang-format; this is what
# makes that true. deps/ is excluded because the vendored copies of Intel's MOC
# and MWSE's SharedSE carry their own style (deps/.clang-format sets
# DisableFormat), and reformatting them would bury real changes in noise.
#
#   pwsh scripts/format.ps1          # rewrite files in place
#   pwsh scripts/format.ps1 -Check   # exit 1 if anything is unformatted
[CmdletBinding()]
param([switch]$Check)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

# clang-format ships with Visual Studio but is usually not on PATH.
$exe = (Get-Command clang-format -ErrorAction SilentlyContinue).Source
if (-not $exe) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $vs = & $vswhere -latest -property installationPath
        $candidate = Join-Path $vs 'VC\Tools\Llvm\bin\clang-format.exe'
        if (Test-Path $candidate) { $exe = $candidate }
    }
}
if (-not $exe) {
    Write-Error 'clang-format not found. Install the "C++ Clang tools for Windows" component, or put clang-format on PATH.'
}

$files = Get-ChildItem -Path (Join-Path $repo 'src'), (Join-Path $repo 'tests') `
    -Include *.cpp, *.h -Recurse -File

if ($Check) {
    # --dry-run -Werror is the supported way to ask. Comparing the formatted
    # text against the file by hand gets line endings and the trailing newline
    # wrong and reports every file as dirty.
    # Windows PowerShell wraps a native command's stderr in an ErrorRecord,
    # which the script-wide 'Stop' preference would turn into a terminating
    # error on the first unformatted file. Relax it just for the loop and read
    # the exit code instead.
    $bad = @()
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    foreach ($f in $files) {
        $null = & $exe -style=file --dry-run -Werror $f.FullName 2>&1
        if ($LASTEXITCODE -ne 0) {
            $bad += $f.FullName.Substring($repo.Length + 1)
        }
    }
    $ErrorActionPreference = $prev
    if ($bad) {
        Write-Host "unformatted ($($bad.Count)):"
        $bad | ForEach-Object { Write-Host "  $_" }
        exit 1
    }
    Write-Host "all $($files.Count) file(s) formatted"
    exit 0
}

& $exe -style=file -i $files.FullName
Write-Host "formatted $($files.Count) file(s)"
