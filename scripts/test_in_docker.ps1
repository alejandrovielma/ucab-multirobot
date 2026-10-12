# Windows wrapper: builds the test image and runs scripts/test_in_docker.sh in it.
# Usage: powershell -File scripts/test_in_docker.ps1 [-NoBuild]
param(
    [switch]$NoBuild,
    [string]$Image = "phineas-ros-test:jazzy"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

if (-not $NoBuild) {
    docker build -t $Image -f (Join-Path $repo "docker/Dockerfile") (Join-Path $repo "docker")
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

docker run --rm -v "${repo}:/ws:ro" -w /ws $Image bash /ws/scripts/test_in_docker.sh
exit $LASTEXITCODE
