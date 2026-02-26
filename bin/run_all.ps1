# Stop script if any command fails
$ErrorActionPreference = "Stop"

Write-Host "==============================="
Write-Host " Starting Flash + Config Flow"
Write-Host "==============================="

# Get current script directory
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$configPath = Join-Path $scriptDir "config.json"

# Resolve flash script path
$flashScript  = Join-Path $scriptDir "flash.ps1"
$logPath = Join-Path $scriptDir "log.txt"

# Verify scripts exist
if (-not (Test-Path $flashScript)) {
    Write-Error "flash.ps1 not found at $flashScript"
    exit 1
}

if (-not (Test-Path $configPath)) {
    Write-Error "config.json not found at $configPath"
    exit 1
}

# Read broker type to select provisioning flow
$config = Get-Content -Raw -Path $configPath | ConvertFrom-Json
$brokerType = $config.broker_type
$configuration = $config.configuration

# Select provision script based on broker_type
switch ($brokerType) {
    "mosquitto" { $provisionScript = Join-Path $scriptDir "provision_mosquitto.ps1" }
    "aws" {
        $awsSingleConfigs = @("MXCHIP_Single", "ST67_T01_Single", "ST67_T02_Single")
        if ($configuration -in $awsSingleConfigs) {
            $provisionScript = Join-Path $scriptDir "provision_aws_single.ps1"
        } else {
            $provisionScript = Join-Path $scriptDir "provision_aws_stsafe.ps1"
        }
    }
    default {
        Write-Error "Unsupported broker_type '$brokerType'. Use 'mosquitto' or 'aws' in config.json."
        exit 1
    }
}

if (-not (Test-Path $provisionScript)) {
    Write-Error "Provision script not found at $provisionScript"
    exit 1
}

# Run flash
Write-Host "`n--- Running Flash Script ---"
& $flashScript

if ($LASTEXITCODE -ne 0) {
    Write-Error "Flash failed. Aborting."
    exit $LASTEXITCODE
}

Write-Host "Flash completed successfully."

# Optional small delay
Start-Sleep -Seconds 2

# Run provision
Write-Host "`n--- Running Provision Script ($brokerType) ---"
# Show live output and append the same output to log.txt
& $provisionScript *>&1 | Tee-Object -FilePath $logPath -Append

if ($LASTEXITCODE -ne 0) {
    Write-Error "Provision failed."
    exit $LASTEXITCODE
}

Write-Host "`nAll steps completed successfully."
