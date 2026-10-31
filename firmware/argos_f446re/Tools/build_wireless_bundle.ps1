param(
    [switch]$SkipEspCompile
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$cubeRoot = 'C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins'
$gnuPlugin = Get-ChildItem -LiteralPath $cubeRoot -Directory `
    -Filter 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*' |
    Sort-Object Name -Descending | Select-Object -First 1
$makePlugin = Get-ChildItem -LiteralPath $cubeRoot -Directory `
    -Filter 'com.st.stm32cube.ide.mcu.externaltools.make.win32*' |
    Sort-Object Name | Select-Object -First 1
if (-not $gnuPlugin -or -not $makePlugin) {
    throw 'STM32CubeIDE GNU/make tools were not found.'
}

$toolBin = Join-Path $gnuPlugin.FullName 'tools\bin'
$makeBin = Join-Path $makePlugin.FullName 'tools\bin'
$gcc = Join-Path $toolBin 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $toolBin 'arm-none-eabi-objcopy.exe'
$size = Join-Path $toolBin 'arm-none-eabi-size.exe'
$make = Join-Path $makeBin 'make.exe'
$env:PATH = "$toolBin;$makeBin;$env:PATH"

$bootRoot = Join-Path $projectRoot 'Bootloader'
$bootBuild = Join-Path $bootRoot 'build'
New-Item -ItemType Directory -Force -Path $bootBuild | Out-Null
$cmsisDevice = Join-Path $projectRoot 'Drivers\CMSIS\Device\ST\STM32F4xx\Include'
$cmsisCore = Join-Path $projectRoot 'Drivers\CMSIS\Include'
$common = @(
    '-mcpu=cortex-m4', '-mthumb', '-Os', '-ffunction-sections', '-fdata-sections',
    '-Wall', '-Wextra', '-DSTM32F446xx', "-I$cmsisDevice", "-I$cmsisCore"
)
& $gcc @common -c (Join-Path $bootRoot 'bootloader.c') -o (Join-Path $bootBuild 'bootloader.o')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $gcc @common -c (Join-Path $bootRoot 'bootloader_startup.s') -o (Join-Path $bootBuild 'bootloader_startup.o')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $gcc '-mcpu=cortex-m4' '-mthumb' '-nostdlib' '-Wl,--gc-sections' `
    "-T$(Join-Path $bootRoot 'bootloader.ld')" `
    (Join-Path $bootBuild 'bootloader_startup.o') (Join-Path $bootBuild 'bootloader.o') `
    '-lgcc' "-Wl,-Map=$(Join-Path $bootBuild 'argos_bootloader.map')" `
    '-o' (Join-Path $bootBuild 'argos_bootloader.elf')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $objcopy '-O' 'binary' (Join-Path $bootBuild 'argos_bootloader.elf') `
    (Join-Path $bootBuild 'argos_bootloader.bin')
& $size (Join-Path $bootBuild 'argos_bootloader.elf')

$debugRoot = Join-Path $projectRoot 'Debug'
Push-Location $debugRoot
try {
    & $make '-j8' 'all'
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $gcc '-o' 'ARGOS_OTA.elf' '@objects.list' '-mcpu=cortex-m4' `
        '-T..\STM32F446RETX_OTA_APP.ld' '--specs=nosys.specs' `
        '-Wl,-Map=ARGOS_OTA.map' '-Wl,--gc-sections' '-static' '--specs=nano.specs' `
        '-mfpu=fpv4-sp-d16' '-mfloat-abi=hard' '-mthumb' `
        '-Wl,--start-group' '-lc' '-lm' '-Wl,--end-group'
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $objcopy '-O' 'binary' 'ARGOS_OTA.elf' 'ARGOS_OTA.bin'
    & $size 'ARGOS_OTA.elf'
}
finally {
    Pop-Location
}

$python = 'C:\Users\yigit\AppData\Local\spyder-6\envs\spyder-runtime\python.exe'
if (-not (Test-Path -LiteralPath $python)) { $python = 'python' }
& $python (Join-Path $PSScriptRoot 'package_wireless_firmware.py') `
    '--bootloader' (Join-Path $bootBuild 'argos_bootloader.bin') `
    '--application' (Join-Path $debugRoot 'ARGOS_OTA.bin') `
    '--output' (Join-Path $debugRoot 'ARGOS_BOOTSTRAP.bin') `
    '--manifest' (Join-Path $debugRoot 'ARGOS_OTA_manifest.json')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if (-not $SkipEspCompile) {
    $arduinoCli = 'C:\Users\yigit\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
    $arduinoConfig = 'C:\Users\yigit\.arduinoIDE\arduino-cli.yaml'
    if ((Test-Path -LiteralPath $arduinoCli) -and (Test-Path -LiteralPath $arduinoConfig)) {
        & $arduinoCli '--config-file' $arduinoConfig 'compile' '--fqbn' 'esp32:esp32:esp32c3' `
            '--output-dir' (Join-Path $projectRoot 'ESP32\build_ota') `
            (Join-Path $projectRoot 'ESP32\esp32_c3_argos_bridge')
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
}

Write-Output 'ARGOS wireless bundle build complete.'
