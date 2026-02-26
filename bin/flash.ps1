# PowerShell script for flashing STM32

# Resolve paths relative to this script location
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$configPath = Join-Path $scriptDir "config.json"

# Bootloader image path
$BOOTLOADER_HEX = Join-Path $scriptDir "Bootloader\B-U585I-IOT02A_BootLoader.hex"

# Ensure config file exists
if (-not (Test-Path -LiteralPath $configPath -PathType Leaf)) {
    Write-Error "config.json not found at: $configPath"
    exit 1
}

# Read selected firmware configuration from config.json
$config = Get-Content -Raw -Path $configPath | ConvertFrom-Json
$configuration = $config.configuration

# Validate required configuration field
if ([string]::IsNullOrWhiteSpace($configuration)) {
    Write-Error "Missing 'configuration' in config.json. Example: `"configuration`": `"MXCHIP_STSAFEA110`""
    exit 1
}

# Build application HEX path from selected configuration
$APP_HEX = Join-Path $scriptDir "$configuration\b_u585_iota02_iot_reference.hex"

# STM32CubeProgrammer path (Windows)
$PROGRAMMER = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

# Check programmer exists
if ([string]::IsNullOrWhiteSpace($PROGRAMMER)) {
    Write-Error "PROGRAMMER path is empty. Set `$PROGRAMMER to STM32_Programmer_CLI.exe full path."
    exit 1
}

if (-not (Test-Path -LiteralPath $PROGRAMMER -PathType Leaf)) {
    Write-Error "STM32CubeProgrammer CLI not found at: $PROGRAMMER"
    exit 1
}

# Check selected application HEX exists
if (-not (Test-Path -LiteralPath $APP_HEX -PathType Leaf)) {
    $availableConfigs = Get-ChildItem -Path $scriptDir -Directory | Where-Object { $_.Name -ne "Bootloader" } | Select-Object -ExpandProperty Name
    Write-Error "App HEX not found for configuration '$configuration': $APP_HEX`nAvailable configurations: $($availableConfigs -join ', ')"
    exit 1
}

# Confirm mass erase
$confirmation = Read-Host "Mass erase will delete all flash on the target. Continue? [y/N]"
if ($confirmation -notin @("y", "Y", "yes", "YES")) {
    Write-Host "Operation cancelled by user."
    exit 1
}

# Perform a mass erase
Write-Host "Performing mass erase..."
& "$PROGRAMMER" -c port=SWD -e all

# Flash hex files using embedded addresses
Write-Host "Flashing $BOOTLOADER_HEX"
& "$PROGRAMMER" -c port=SWD -w "$BOOTLOADER_HEX"

Write-Host "Flashing $APP_HEX"
& "$PROGRAMMER" -c port=SWD -w "$APP_HEX"

# Reset the target
Write-Host "Issuing device reset..."
& "$PROGRAMMER" -c port=SWD -rst

# Final delay
Start-Sleep -Seconds 1
