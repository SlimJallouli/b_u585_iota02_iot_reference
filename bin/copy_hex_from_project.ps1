# Copies all .hex files from .\project into .\bin, preserving subfolder structure.

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$projectDir = Join-Path $repoRoot "project"
$binDir = $scriptDir

if (-not (Test-Path -LiteralPath $projectDir -PathType Container)) {
    Write-Error "Project folder not found: $projectDir"
    exit 1
}

$hexFiles = Get-ChildItem -Path $projectDir -Recurse -File -Filter *.hex

if (-not $hexFiles) {
    Write-Host "No .hex files found under: $projectDir"
    exit 0
}

$copiedCount = 0

foreach ($hex in $hexFiles) {
    $relativeDir = $hex.DirectoryName.Substring($projectDir.Length).TrimStart('\', '/')
    $targetDir = if ([string]::IsNullOrWhiteSpace($relativeDir)) { $binDir } else { Join-Path $binDir $relativeDir }

    if (-not (Test-Path -LiteralPath $targetDir -PathType Container)) {
        New-Item -ItemType Directory -Path $targetDir -Force | Out-Null
    }

    $targetPath = Join-Path $targetDir $hex.Name
    Copy-Item -LiteralPath $hex.FullName -Destination $targetPath -Force
    $copiedCount++
    Write-Host "Copied: $($hex.FullName) -> $targetPath"
}

Write-Host "Done. Copied $copiedCount .hex file(s) into: $binDir"
