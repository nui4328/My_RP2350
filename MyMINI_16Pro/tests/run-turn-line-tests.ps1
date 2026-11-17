param([string]$BuildDirectory = (Join-Path $env:TEMP 'MyMINI_16Pro-turn-line-tests'))
$ErrorActionPreference = 'Stop'
$libraryRoot = Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$testExe = Join-Path $BuildDirectory 'turn_line_capture_tests.exe'
& g++ -std=c++11 -Wall -Wextra -Werror "-I$libraryRoot\src" (Join-Path $PSScriptRoot 'turn_line_capture_tests.cpp') -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Turn-line host test compilation failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Turn-line host behavior tests failed' }
