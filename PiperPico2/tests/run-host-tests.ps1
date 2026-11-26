$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force .build | Out-Null
    & g++ -std=c++17 -Wall -Wextra -Werror -I tests/host -I . PiperPico2.cpp PiperPico2Motor.cpp PiperPico2Calibration.cpp PiperPico2Encoder.cpp tests/host/core_tests.cpp -o .build/core_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & .build/core_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host behavior tests failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -I tests/host -I . PiperPico2.cpp PiperPico2Motor.cpp PiperPico2Calibration.cpp PiperPico2Encoder.cpp tests/host/ui_tests.cpp -o .build/ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'OLED UI test compilation failed' }
    & .build/ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'OLED UI behavior tests failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -I tests/host -I . PiperPico2.cpp PiperPico2Motor.cpp PiperPico2Calibration.cpp PiperPico2Encoder.cpp tests/host/bno_ui_tests.cpp -o .build/bno_ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'BNO055 UI test compilation failed' }
    & .build/bno_ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'BNO055 UI behavior tests failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -I tests/host -I . PiperPico2.cpp PiperPico2Motor.cpp PiperPico2Calibration.cpp PiperPico2Encoder.cpp tests/host/tcs_ui_tests.cpp -o .build/tcs_ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'TCS UI test compilation failed' }
    & .build/tcs_ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'TCS UI behavior tests failed' }
    & python tests/test_tcs_color.py
    if ($LASTEXITCODE -ne 0) { throw 'Installed my_TCS classification tests failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -I tests/host -I . PiperPico2.cpp PiperPico2Motor.cpp PiperPico2Calibration.cpp PiperPico2Encoder.cpp tests/host/cold_boot_ui_tests.cpp -o .build/cold_boot_ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Cold boot UI test compilation failed' }
    & .build/cold_boot_ui_tests.exe
    if ($LASTEXITCODE -ne 0) { throw 'Cold boot UI behavior tests failed' }





} finally { Pop-Location }
