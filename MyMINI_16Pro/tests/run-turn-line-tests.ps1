param([string]$BuildDirectory = (Join-Path $env:TEMP 'MyMINI_16Pro-turn-line-tests'))
$ErrorActionPreference = 'Stop'
$libraryRoot = Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Path $BuildDirectory -Force | Out-Null
$testExe = Join-Path $BuildDirectory 'turn_line_capture_tests.exe'
& g++ -std=c++11 -Wall -Wextra -Werror "-I$libraryRoot\src" (Join-Path $PSScriptRoot 'turn_line_capture_tests.cpp') -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Turn-line host test compilation failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Turn-line host behavior tests failed' }

$whiteTestExe = Join-Path $BuildDirectory 'line_crossing_white_tests.exe'
& g++ -std=c++11 -Wall -Wextra -Werror "-I$libraryRoot\src" (Join-Path $PSScriptRoot 'line_crossing_white_tests.cpp') -o $whiteTestExe
if ($LASTEXITCODE -ne 0) { throw 'Line crossing white-release test compilation failed' }
& $whiteTestExe
if ($LASTEXITCODE -ne 0) { throw 'Line crossing white-release behavior tests failed' }

$edgeTestExe = Join-Path $BuildDirectory 'turn_front_edge_tests.exe'
& g++ -std=c++11 -Wall -Wextra -Werror "-I$libraryRoot\src" (Join-Path $PSScriptRoot 'turn_front_edge_tests.cpp') -o $edgeTestExe
if ($LASTEXITCODE -ne 0) { throw 'Turn front-edge test compilation failed' }
& $edgeTestExe
if ($LASTEXITCODE -ne 0) { throw 'Turn front-edge behavior tests failed' }
