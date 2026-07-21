# Fetches the SingleStepTests 680x0 suite (68000 JSON tests) into tests/data.
# The harness (openmac_sst) takes the 68000/v1 directory as its argument:
#   openmac_sst tests/data/680x0/68000/v1
param(
    [string]$Dest = (Join-Path $PSScriptRoot 'data')
)
$repo = Join-Path $Dest '680x0'
if (Test-Path (Join-Path $repo '68000\v1')) {
    Write-Host "SingleStepTests already present at $repo"
    exit 0
}
New-Item -ItemType Directory -Force $Dest | Out-Null
git clone --depth 1 https://github.com/SingleStepTests/680x0.git $repo
