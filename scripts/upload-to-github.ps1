# =====================================================================
# 一键上传本仓库到 GitHub
# 用法：在仓库根目录运行  powershell -ExecutionPolicy Bypass -File scripts\upload-to-github.ps1
# 或在 VS Code 终端中：  ./scripts/upload-to-github.ps1
#
# 脚本会依次完成：
#   1. 检查 gh (GitHub CLI) 是否安装，未安装则用 winget 安装
#   2. 检查登录状态，未登录则打开浏览器让你完成一次 GitHub 授权
#   3. 创建一个【私有】仓库并推送到 main 分支
# =====================================================================

$ErrorActionPreference = "Stop"

# ---------- 配置区（可按需修改）----------
$RepoName   = "esp32-car-course-design"          # GitHub 仓库名
$Visibility = "private"                          # private=私有（推荐） / public=公开
$Description = "电子技术课程设计：ESP32-S3 三轮麦克纳姆轮小车（循迹/避障/科目三找球踢球）"
# -----------------------------------------

Set-Location (Split-Path $PSScriptRoot -Parent)   # 切到仓库根目录

Write-Host "==> 1/4 检查 GitHub CLI (gh) ..." -ForegroundColor Cyan
if (-not (Get-Command gh -ErrorAction SilentlyContinue)) {
    Write-Host "    未安装 gh，正在用 winget 安装（需几分钟）..." -ForegroundColor Yellow
    winget install --id GitHub.cli -e --accept-source-agreements --accept-package-agreements --silent
    $env:Path = [System.Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [System.Environment]::GetEnvironmentVariable('Path','User')
}
gh --version

Write-Host "`n==> 2/4 检查登录状态 ..." -ForegroundColor Cyan
gh auth status *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Host "    未登录，即将打开浏览器完成 GitHub 授权（浏览器会弹出页面，点 Authorize 即可）..." -ForegroundColor Yellow
    gh auth login --hostname github.com --git-protocol https --web
    if ($LASTEXITCODE -ne 0) { throw "GitHub 登录失败，请重试" }
}
$user = gh api user --jq .login
Write-Host "    已登录：$user" -ForegroundColor Green

Write-Host "`n==> 3/4 创建仓库并推送 ..." -ForegroundColor Cyan
gh repo create "$RepoName" --$Visibility --source . --remote origin --push --description $Description
if ($LASTEXITCODE -ne 0) { throw "创建/推送仓库失败" }

Write-Host "`n==> 4/4 完成！" -ForegroundColor Green
$url = "https://github.com/$user/$RepoName"
Write-Host "仓库地址：$url"
Write-Host ""
Write-Host "后续把组员加为协作者：仓库页面 -> Settings -> Collaborators -> Add people"
Write-Host "协作规范请看 CONTRIBUTING.md"
