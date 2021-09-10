foreach ($file in get-ChildItem *.h) {
    Write-Output $file.name
    Move-Item $file.name ($file.name + ".tmp")
    Get-Content ($file.name + ".tmp") | Set-Content -Encoding utf8 $file.name
    Remove-Item ($file.name + ".tmp")
}

foreach ($file in get-ChildItem *.c) {
    Write-Output $file.name
    Move-Item $file.name ($file.name + ".tmp")
    Get-Content ($file.name + ".tmp") | Set-Content -Encoding utf8 $file.name
    Remove-Item ($file.name + ".tmp")
 }