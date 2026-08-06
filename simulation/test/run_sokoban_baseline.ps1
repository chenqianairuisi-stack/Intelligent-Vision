param(
    [string]$MapGlob = "map\map_bomb\*.txt",
    [string]$OutputMd = "test\sokoban_baseline.md",
    [string]$OutputCsv = "",
    [int]$MaxMappingsPerMap = 24,
    [switch]$Build
)

$ErrorActionPreference = "Stop"

$Root = Resolve-Path (Join-Path $PSScriptRoot "..")
$RepoRoot = Resolve-Path (Join-Path $Root "..")
$Solver = Join-Path $Root "solver.exe"
$SolverWorkDir = $Root
$MapInput = Join-Path $SolverWorkDir "map_input.txt"
$PathOutput = Join-Path $SolverWorkDir "path_output.txt"
if ($OutputCsv) {
    $OutputMd = [System.IO.Path]::ChangeExtension($OutputCsv, ".md")
}
if ([System.IO.Path]::IsPathRooted($OutputMd)) {
    $OutputPath = $OutputMd
} else {
    $OutputPath = Join-Path $Root $OutputMd
}

function Get-Permutations {
    param([int[]]$Items)

    if ($Items.Count -le 1) {
        ,$Items
        return
    }

    for ($i = 0; $i -lt $Items.Count; $i++) {
        $head = $Items[$i]
        $tail = @()
        for ($j = 0; $j -lt $Items.Count; $j++) {
            if ($j -ne $i) { $tail += $Items[$j] }
        }
        foreach ($perm in Get-Permutations -Items $tail) {
            ,(@($head) + @($perm))
        }
    }
}

function Read-FirstLineTokens {
    param([string[]]$Lines, [string]$Prefix)

    foreach ($line in $Lines) {
        if ($line.StartsWith($Prefix)) {
            return $line.Split(" ", [System.StringSplitOptions]::RemoveEmptyEntries)
        }
    }
    return @()
}

function ConvertTo-MdCell {
    param($Value)

    $text = [string]$Value
    return $text.Replace("\", "\\").Replace("|", "\|").Replace("`r", " ").Replace("`n", " ")
}

if ($Build) {
    Push-Location $RepoRoot
    try {
        g++ -std=c++17 -O3 -Iproject\Core -Iproject\Algorithm -Iproject\App `
            simulation\main.cpp `
            project\Algorithm\Sokoban.cpp project\Algorithm\Exploration.cpp `
            project\Algorithm\PlanningCommon.cpp project\Algorithm\Strategy.cpp `
            project\Algorithm\MacroPlanner.cpp -o simulation\solver.exe
    } finally {
        Pop-Location
    }
}

if (!(Test-Path $Solver)) {
    throw "solver.exe not found: $Solver"
}
if (!(Test-Path $SolverWorkDir)) {
    throw "solver work dir not found: $SolverWorkDir"
}

$outputDir = Split-Path $OutputPath -Parent
if ($outputDir -and !(Test-Path $outputDir)) {
    New-Item -ItemType Directory -Path $outputDir | Out-Null
}

$savedInput = if (Test-Path $MapInput) { Get-Content $MapInput -Raw } else { $null }
$savedOutput = if (Test-Path $PathOutput) { Get-Content $PathOutput -Raw } else { $null }
$rows = @()

try {
    $maps = Get-ChildItem -Path (Join-Path $Root $MapGlob) | Sort-Object FullName
    foreach ($mapFile in $maps) {
        $mapLines = Get-Content $mapFile.FullName
        $boxCount = (($mapLines -join "").ToCharArray() | Where-Object { $_ -eq '$' }).Count
        $targetCount = (($mapLines -join "").ToCharArray() | Where-Object { $_ -eq '.' }).Count

        if ($boxCount -le 0 -or $boxCount -ne $targetCount) {
            continue
        }

        $ids = 0..($boxCount - 1)
        $mappings = @(Get-Permutations -Items $ids | Select-Object -First $MaxMappingsPerMap)

        foreach ($mapping in $mappings) {
            $mappingText = ($mapping -join " ")
            Set-Content -Path $MapInput -Value ($mapLines + $mappingText) -Encoding ascii

            $sw = [System.Diagnostics.Stopwatch]::StartNew()
            Push-Location $SolverWorkDir
            try {
                & $Solver | Out-Null
            } finally {
                Pop-Location
            }
            $sw.Stop()

            $outLines = Get-Content $PathOutput
            $timeTokens = Read-FirstLineTokens -Lines $outLines -Prefix "TIMES "
            $profileTokens = Read-FirstLineTokens -Lines $outLines -Prefix "PROFILE "
            $sokobanLine = ($outLines | Select-String -Pattern "^SOKOBAN$" | Select-Object -First 1).LineNumber
            $failed = [bool]($outLines | Select-String -Pattern "^FAILED$" | Select-Object -First 1)
            $sokobanSteps = if ($sokobanLine) { $outLines.Count - $sokobanLine } else { -1 }

            $rows += [pscustomobject]@{
                map = $mapFile.Name
                mapping = $mappingText
                failed = $failed
                elapsed_ms = $sw.ElapsedMilliseconds
                phase1_bomb_ms = if ($timeTokens.Count -ge 5) { [int]$timeTokens[1] } else { -1 }
                patrol_ms = if ($timeTokens.Count -ge 5) { [int]$timeTokens[2] } else { -1 }
                phase2_bomb_ms = if ($timeTokens.Count -ge 5) { [int]$timeTokens[3] } else { -1 }
                sokoban_ms = if ($timeTokens.Count -ge 5) { [int]$timeTokens[4] } else { -1 }
                sokoban_steps = $sokobanSteps
                expanded_nodes = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[1] } else { 0 }
                generated_moves = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[2] } else { 0 }
                tt_hits = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[3] } else { 0 }
                heuristic_dead_prunes = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[4] } else { 0 }
                threshold_prunes = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[5] } else { 0 }
                path_cycle_prunes = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[6] } else { 0 }
                static_deadlock_prunes = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[7] } else { 0 }
                block_2x2_prunes = if ($profileTokens.Count -ge 13) { [uint32]$profileTokens[8] } else { 0 }
                max_depth = if ($profileTokens.Count -ge 13) { [uint16]$profileTokens[9] } else { 0 }
                threshold_iterations = if ($profileTokens.Count -ge 13) { [uint16]$profileTokens[10] } else { 0 }
                final_threshold = if ($profileTokens.Count -ge 13) { [uint16]$profileTokens[11] } else { 0 }
                nps = if ($profileTokens.Count -ge 13) { [uint64]$profileTokens[12] } else { 0 }
            }
        }
    }

    $failedRows = @($rows | Where-Object { $_.failed })
    $outputDir = Split-Path $OutputPath -Parent
    if ($outputDir -and !(Test-Path $outputDir)) {
        New-Item -ItemType Directory -Path $outputDir | Out-Null
    }

    $columns = @(
        "map",
        "mapping",
        "failed",
        "elapsed_ms",
        "phase1_bomb_ms",
        "patrol_ms",
        "phase2_bomb_ms",
        "sokoban_ms",
        "sokoban_steps",
        "expanded_nodes",
        "generated_moves",
        "tt_hits",
        "heuristic_dead_prunes",
        "threshold_prunes",
        "path_cycle_prunes",
        "static_deadlock_prunes",
        "block_2x2_prunes",
        "max_depth",
        "threshold_iterations",
        "final_threshold",
        "nps"
    )

    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add("# Sokoban Baseline Report")
    $lines.Add("")
    $lines.Add("- Map glob: ``$MapGlob``")
    $lines.Add("- Total cases: $($rows.Count)")
    $lines.Add("- Failed cases: $($failedRows.Count)")
    $lines.Add("- Max mappings per map: $MaxMappingsPerMap")
    $lines.Add("")

    if ($failedRows.Count -gt 0) {
        $lines.Add("## Failed Cases")
        $lines.Add("")
        $lines.Add("| map | mapping | elapsed_ms | sokoban_steps |")
        $lines.Add("| --- | --- | --- | --- |")
        foreach ($row in $failedRows) {
            $lines.Add("| $(ConvertTo-MdCell $row.map) | $(ConvertTo-MdCell $row.mapping) | $(ConvertTo-MdCell $row.elapsed_ms) | $(ConvertTo-MdCell $row.sokoban_steps) |")
        }
        $lines.Add("")
    }

    $lines.Add("## All Cases")
    $lines.Add("")
    $lines.Add("| " + (($columns | ForEach-Object { ConvertTo-MdCell $_ }) -join " | ") + " |")
    $lines.Add("| " + (($columns | ForEach-Object { "---" }) -join " | ") + " |")
    foreach ($row in $rows) {
        $values = foreach ($column in $columns) {
            ConvertTo-MdCell $row.$column
        }
        $lines.Add("| " + ($values -join " | ") + " |")
    }

    Set-Content -Path $OutputPath -Value $lines -Encoding utf8
    Write-Host "Wrote baseline: $OutputPath"
} finally {
    if ($null -ne $savedInput) {
        Set-Content -Path $MapInput -Value $savedInput -NoNewline
    }
    if ($null -ne $savedOutput) {
        Set-Content -Path $PathOutput -Value $savedOutput -NoNewline
    }
}

