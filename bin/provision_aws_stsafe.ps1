# Load configuration
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$configPath = Join-Path $scriptDir "config.json"
$config = Get-Content -Raw -Path $configPath | ConvertFrom-Json
$brokerType = $config.broker_type
$configuration = $config.configuration

# Guard: this script is AWS-only
if ($brokerType -ne "aws") {
    throw "This script supports broker_type 'aws' only. Current value: '$brokerType'."
}

# Guard: validate STSAFE-compatible firmware profiles
$awsStsafeCompatibleConfigs = @("MXCHIP_STSAFEA110", "MXCHIP_STSAFEA120", "ST67_T02_STSAFEA110", "ST67_T02_STSAFEA120")
if ($configuration -notin $awsStsafeCompatibleConfigs) {
    throw "Configuration '$configuration' is not compatible with provision_aws_stsafe.ps1. Allowed values: $($awsStsafeCompatibleConfigs -join ', ')"
}

# Auto-detect STLink Virtual COM Port
$portName = Get-CimInstance Win32_SerialPort | Where-Object { $_.Name -like "*STLink Virtual COM Port*" } | Select-Object -ExpandProperty DeviceID

if (-not $portName) {
    throw "STLink Virtual COM Port not found. Connect the board and verify the STLink COM port is present in Device Manager."
}

$baudRate = 115200
$serialPort = $null

# Root CA download settings
$defaultAwsRootCaUrl = "https://www.amazontrust.com/repository/SFC2CA-SFSRootCAG2.pem"
$awsRootCaUrl = $defaultAwsRootCaUrl
$awsRootCaPath = Join-Path $scriptDir "SFSRootCAG2.pem"
$awsMqttPort = 8883

# AWS IoT policy to attach to device certificate
$awsPolicyName = $config.aws_policy_name
if ([string]::IsNullOrWhiteSpace($awsPolicyName)) {
    $awsPolicyName = "AllowAllDev"
}

function Send-Command {
    param (
        [string]$command,
        [int]$timeoutMs = 1500
    )

    $serialPort.WriteLine($command)

    $response = ""
    $sw = [System.Diagnostics.Stopwatch]::StartNew()

    while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
        if ($serialPort.BytesToRead -gt 0) {
            $response += $serialPort.ReadExisting()
        }
        Start-Sleep -Milliseconds 50
    }

    return $response
}

# Send a full PEM/text file over the CLI serial session
function Send-FileContent {
    param ([string]$filePath)
    $content = Get-Content -Raw -Path $filePath
    $serialPort.WriteLine($content)
    Start-Sleep -Milliseconds 500
}

# Extract a PEM block from command response text
function Get-PemBlock {
    param(
        [string]$Text,
        [string]$BeginMarker,
        [string]$EndMarker
    )

    $pattern = [Regex]::Escape($BeginMarker) + "(.|\s)*?" + [Regex]::Escape($EndMarker)
    if ($Text -match $pattern) {
        return $matches[0].Trim()
    }

    return $null
}

# Download root CA file before importing to device
function Ensure-RootCaFile {
    param(
        [string]$Url,
        [string]$OutPath
    )

    Write-Host "Downloading root CA: $Url"
    try {
        Invoke-WebRequest -Uri $Url -OutFile $OutPath -UseBasicParsing
    }
    catch {
        throw "Failed to download root CA from '$Url': $($_.Exception.Message)"
    }
}

function Invoke-AwsCli {
    param(
        [string[]]$CommandArgs,
        [switch]$AllowFailure
    )

    if (-not (Get-Command aws -ErrorAction SilentlyContinue)) {
        throw "AWS CLI not found. Install and configure AWS CLI first (aws configure)."
    }

    if ($null -eq $CommandArgs -or $CommandArgs.Count -eq 0) {
        throw "Invoke-AwsCli called without command arguments."
    }

    Write-Host "Running AWS CLI: aws $($CommandArgs -join ' ')"

    $stdoutFile = [System.IO.Path]::GetTempFileName()
    $stderrFile = [System.IO.Path]::GetTempFileName()
    try {
        $process = Start-Process -FilePath "aws" -ArgumentList $CommandArgs -NoNewWindow -Wait -PassThru -RedirectStandardOutput $stdoutFile -RedirectStandardError $stderrFile
        $stdout = ""
        $stderr = ""

        if (Test-Path -LiteralPath $stdoutFile) {
            $stdoutRaw = Get-Content -Raw -Path $stdoutFile -ErrorAction SilentlyContinue
            if ($null -ne $stdoutRaw) {
                $stdout = ([string]$stdoutRaw).Trim()
            }
        }
        if (Test-Path -LiteralPath $stderrFile) {
            $stderrRaw = Get-Content -Raw -Path $stderrFile -ErrorAction SilentlyContinue
            if ($null -ne $stderrRaw) {
                $stderr = ([string]$stderrRaw).Trim()
            }
        }

        if (-not [string]::IsNullOrWhiteSpace($stdout)) {
            Write-Host "AWS CLI stdout:"
            Write-Host $stdout
        }
        if (-not [string]::IsNullOrWhiteSpace($stderr)) {
            Write-Host "AWS CLI stderr:"
            Write-Host $stderr
        }

        if (-not $AllowFailure -and $process.ExitCode -ne 0) {
            throw "AWS CLI command failed: aws $($CommandArgs -join ' ')`n$stderr`n$stdout"
        }

        return [PSCustomObject]@{
            ExitCode = $process.ExitCode
            Output   = "$stdout`n$stderr".Trim()
            StdOut   = $stdout
            StdErr   = $stderr
        }
    }
    finally {
        if (Test-Path -LiteralPath $stdoutFile) { Remove-Item -LiteralPath $stdoutFile -Force -ErrorAction SilentlyContinue }
        if (Test-Path -LiteralPath $stderrFile) { Remove-Item -LiteralPath $stderrFile -Force -ErrorAction SilentlyContinue }
    }
}

function Ensure-AwsThing {
    param(
        [string]$ThingName
    )

    $describe = Invoke-AwsCli -CommandArgs @("iot", "describe-thing", "--thing-name", $ThingName) -AllowFailure
    if ($describe.ExitCode -eq 0) {
        return
    }

    Write-Host "Creating AWS IoT Thing: $ThingName"
    Invoke-AwsCli -CommandArgs @("iot", "create-thing", "--thing-name", $ThingName) | Out-Null
}

function Get-ThingPrincipalArn {
    param(
        [string]$ThingName
    )

    $result = Invoke-AwsCli -CommandArgs @("iot", "list-thing-principals", "--thing-name", $ThingName, "--output", "json") -AllowFailure
    if ($result.ExitCode -ne 0 -or [string]::IsNullOrWhiteSpace($result.StdOut)) {
        return $null
    }

    try {
        $json = $result.StdOut | ConvertFrom-Json
        if ($null -ne $json.principals -and $json.principals.Count -gt 0) {
            return [string]$json.principals[0]
        }
    }
    catch {
        return $null
    }

    return $null
}

function Ensure-AttachThingPrincipal {
    param(
        [string]$ThingName,
        [string]$PrincipalArn
    )

    $result = Invoke-AwsCli -CommandArgs @("iot", "attach-thing-principal", "--thing-name", $ThingName, "--principal", $PrincipalArn) -AllowFailure
    if ($result.ExitCode -eq 0) {
        return
    }

    if ($result.Output -match "already" -or $result.Output -match "ResourceAlreadyExistsException") {
        return
    }

    throw "Failed to attach certificate to thing '$ThingName'."
}

function Ensure-AwsPolicyExists {
    param(
        [string]$PolicyName
    )

    $describe = Invoke-AwsCli -CommandArgs @("iot", "get-policy", "--policy-name", $PolicyName, "--output", "json") -AllowFailure
    if ($describe.ExitCode -eq 0) {
        return
    }

    $policyDoc = @'
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:*",
      "Resource": "*"
    }
  ]
}
'@

    $policyDocFile = [System.IO.Path]::GetTempFileName()
    try {
        $policyDoc | Out-File -Encoding ascii -FilePath $policyDocFile
        $policyDocUri = "file://$($policyDocFile -replace '\\','/')"
        Write-Host "AWS policy '$PolicyName' not found. Creating it with default policy document."
        Invoke-AwsCli -CommandArgs @("iot", "create-policy", "--policy-name", $PolicyName, "--policy-document", $policyDocUri, "--output", "json") | Out-Null
    }
    finally {
        if (Test-Path -LiteralPath $policyDocFile) { Remove-Item -LiteralPath $policyDocFile -Force -ErrorAction SilentlyContinue }
    }
}

function Ensure-AttachPolicy {
    param(
        [string]$PolicyName,
        [string]$TargetArn
    )

    Ensure-AwsPolicyExists -PolicyName $PolicyName

    $result = Invoke-AwsCli -CommandArgs @("iot", "attach-policy", "--policy-name", $PolicyName, "--target", $TargetArn) -AllowFailure
    if ($result.ExitCode -eq 0) {
        return
    }

    if ($result.Output -match "already" -or $result.Output -match "ResourceAlreadyExistsException") {
        return
    }

    throw "Failed to attach policy '$PolicyName' to certificate."
}

function Register-StsafeCertificate {
    param(
        [string]$CertFile,
        [string]$ThingName,
        [string]$PolicyName
    )

    Ensure-AwsThing -ThingName $ThingName

    # If a principal is already attached to this Thing, reuse it (rerun-safe flow).
    $existingPrincipal = Get-ThingPrincipalArn -ThingName $ThingName
    if (-not [string]::IsNullOrWhiteSpace($existingPrincipal)) {
        Write-Host "Thing '$ThingName' already has principal '$existingPrincipal'. Skipping certificate registration."
        Ensure-AttachPolicy -PolicyName $PolicyName -TargetArn $existingPrincipal
        return
    }

    $certUri = "file://$($CertFile -replace '\\','/')"
    Write-Host "Registering STSAFE certificate in AWS IoT..."
    $register = Invoke-AwsCli -CommandArgs @(
        "iot", "register-certificate-without-ca",
        "--certificate-pem", $certUri,
        "--status", "ACTIVE",
        "--output", "json"
    ) -AllowFailure

    if ($register.ExitCode -ne 0) {
        if ($register.Output -match "ResourceAlreadyExistsException") {
            # Cert is already registered in account; if principal is now visible on Thing, continue.
            $existingPrincipal = Get-ThingPrincipalArn -ThingName $ThingName
            if (-not [string]::IsNullOrWhiteSpace($existingPrincipal)) {
                Write-Host "Certificate already registered. Reusing principal '$existingPrincipal'."
                Ensure-AttachPolicy -PolicyName $PolicyName -TargetArn $existingPrincipal
                return
            }
            throw "Certificate already registered in AWS, but no principal is attached to Thing '$ThingName'. Attach it manually, then rerun."
        }
        throw "Failed to register STSAFE certificate in AWS IoT."
    }

    if ([string]::IsNullOrWhiteSpace($register.StdOut)) {
        throw "AWS returned empty response for register-certificate-without-ca."
    }

    $registerJson = $register.StdOut | ConvertFrom-Json
    $certArn = $registerJson.certificateArn
    if ([string]::IsNullOrWhiteSpace($certArn)) {
        throw "AWS did not return certificateArn for the registered certificate."
    }

    Ensure-AttachThingPrincipal -ThingName $ThingName -PrincipalArn $certArn
    Ensure-AttachPolicy -PolicyName $PolicyName -TargetArn $certArn
}

function Get-AwsIotEndpoint {
    $result = Invoke-AwsCli -CommandArgs @("iot", "describe-endpoint", "--endpoint-type", "iot:Data-ATS", "--query", "endpointAddress", "--output", "text")
    $endpoint = $result.Output
    if ([string]::IsNullOrWhiteSpace($endpoint)) {
        throw "Failed to get AWS IoT endpoint using AWS CLI. Empty endpoint returned."
    }

    return $endpoint
}

try {
    # Open serial session to board CLI
    $serialPort = New-Object System.IO.Ports.SerialPort $portName, $baudRate, "None", 8, "One"
    $serialPort.Open()

    # Wi-Fi settings
    Send-Command "conf set wifi_ssid "
    Send-Command "conf set wifi_credential "
    Send-Command "conf commit"
    Send-Command "reset"
    Start-Sleep -Seconds 5

    # Root CA
    Ensure-RootCaFile -Url $awsRootCaUrl -OutPath $awsRootCaPath
    Send-Command "pki import cert root_ca_cert"
    Send-FileContent $awsRootCaPath

    # Thing name
    $thingName = "unknown_device"
    $response = Send-Command "conf get thing_name"
    if ($response -match 'thing_name\s*=\s*"([^"]+)"') {
        $thingName = $matches[1]
        Write-Host "Thing Name detected: $thingName"
    } else {
        Write-Warning "Failed to read thing_name"
    }

    # Read and save TLS certificate
    $certResponse = Send-Command "pki export cert tls_cert" 4000
    $certPem = Get-PemBlock -Text $certResponse -BeginMarker "-----BEGIN CERTIFICATE-----" -EndMarker "-----END CERTIFICATE-----"

    if ($certPem) {
        $certFile = Join-Path $scriptDir "$thingName.cert.pem"
        $certPem | Out-File -Encoding ascii $certFile
        Write-Host "Certificate saved to $certFile"
        Register-StsafeCertificate -CertFile $certFile -ThingName $thingName -PolicyName $awsPolicyName
    } else {
        Write-Warning "Certificate not found in response"
    }

    # MQTT config
    $awsEndpoint = Get-AwsIotEndpoint
    Write-Host "Using AWS IoT endpoint from AWS CLI: $awsEndpoint"
    Send-Command "conf set mqtt_endpoint $awsEndpoint"
    Send-Command "conf set mqtt_port $awsMqttPort"
    Send-Command "conf set wifi_ssid $($config.wifi_ssid)"
    Send-Command "conf set wifi_credential $($config.wifi_credential)"
    Send-Command "conf commit"
    Start-Sleep -Seconds 1
    Send-Command "reset"
}
finally {
    # Always close serial port, even on failure
    if ($null -ne $serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
        Write-Host "Serial port closed."
    }
}
