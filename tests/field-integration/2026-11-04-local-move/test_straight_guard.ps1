$ErrorActionPreference = 'Stop'
# Execute the header's numeric function on the host after only primitive-type
# and signature translation to C#. This does not test HAL or real motor outputs.
$header = Get-Content -LiteralPath (Join-Path $PSScriptRoot '../Core/Inc/argos_straight_guard.h') -Raw
$start = $header.IndexOf('static inline uint32_t Argos_StraightDecision')
$end = $header.IndexOf('#endif', $start)
if ($start -lt 0 -or $end -lt 0) { throw 'Guard function boundary not found' }
$function = $header.Substring($start, $end-$start).Trim()
$function = $function.Replace('static inline uint32_t Argos_StraightDecision', 'public static uint Decide')
$function = $function.Replace('uint32_t', 'uint').Replace('int32_t', 'int')
Add-Type -TypeDefinition "public static class ArgosStraightGuardHost { $function }"
$cases = @(
    @(0,0,0,0,0), @(0,0,5001,0,1), @(-31,0,100,0,2),
    @(1151,0,100,0,3), @(300,0,100,0,4), @(950,950,100,0,5),
    @(100,50,100,0,256), @(50,100,100,0,512),
    @(100,80,100,256,256), @(100,90,100,256,0), @(960,940,100,0,256)
)
foreach ($case in $cases) {
    $actual = [ArgosStraightGuardHost]::Decide([int]$case[0],[int]$case[1],[uint32]$case[2],[uint32]$case[3])
    if ($actual -ne $case[4]) { throw "Guard mismatch: $case => $actual" }
}
$left = 0; $right = 0; $hold = 0; $complete = $false; $largestDifference = 0
for ($step = 0; $step -lt 250; $step++) {
    $hold = [ArgosStraightGuardHost]::Decide($left,$right,($step*20),$hold)
    $reason = $hold -band 255
    if ($reason -eq 5) { $complete = $true; break }
    if ($reason -ne 0) { throw "Asymmetric simulation aborted: $reason" }
    if (($hold -band 256) -eq 0) { $left += 8 }
    if (($hold -band 512) -eq 0) { $right += 18 }
    $largestDifference = [Math]::Max($largestDifference,[Math]::Abs($left-$right))
}
if (-not $complete -or $largestDifference -gt 70) { throw 'Synchronization simulation failed' }
Write-Output "PASS: $($cases.Count) numeric cases and unequal-speed synchronization; max progress difference $largestDifference/1000. HAL/motion validation pending."
