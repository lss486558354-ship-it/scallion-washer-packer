<#==============================================================================
  GitHub Pages 上传脚本
  用法：
    1. 先在 GitHub 创建仓库（例如：scallion-flowchart）
    2. 克隆仓库到本地
    3. 修改下面的 $REPO_PATH 为你的仓库本地路径
    4. 运行脚本: .\upload_github_pages.ps1
==============================================================================#>

# ========== 配置区（请修改）==========
$REPO_PATH = "C:\Users\86158\Downloads\0\2026_04_05_ver_01.00.00\scallion-flowchart"  # 改成你的仓库克隆路径
$GH_USERNAME = "is900"                # GitHub用户名
$REPO_NAME = "scallion-flowchart"      # 仓库名
# ====================================

$ErrorActionPreference = "Stop"

# 检查路径
if (-not (Test-Path $REPO_PATH)) {
    Write-Host "❌ 仓库路径不存在: $REPO_PATH" -ForegroundColor Red
    Write-Host ""
    Write-Host "请先执行以下步骤：" -ForegroundColor Yellow
    Write-Host "  1. 打开 GitHub: https://github.com"
    Write-Host "  2. 点击右上角 '+' -> 'New repository'"
    Write-Host "  3. Repository name 填入: scallion-flowchart"
    Write-Host "  4. 选择 Public"
    Write-Host "  5. 点击 'Create repository'"
    Write-Host "  6. 复制仓库URL"
    Write-Host "  7. 在本地执行: git clone <仓库URL>"
    Write-Host ""
    Write-Host "克隆完成后，将 $REPO_PATH 改成本地仓库路径" -ForegroundColor Cyan
    exit 1
}

# 复制HTML文件到仓库
$html_source = Join-Path $PSScriptRoot "流程图.html"
$html_dest = Join-Path $REPO_PATH "index.html"

if (-not (Test-Path $html_source)) {
    Write-Host "❌ 源文件不存在: $html_source" -ForegroundColor Red
    exit 1
}

Write-Host "📁 复制文件到仓库..." -ForegroundColor Cyan
Copy-Item $html_source $html_dest -Force

# Git操作
Set-Location $REPO_PATH

Write-Host "📤 提交到Git..." -ForegroundColor Cyan
git add .
git commit -m "Upload flowchart - $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"

Write-Host "🚀 推送到GitHub..." -ForegroundColor Cyan
git push -u origin main

# 生成GitHub Pages URL
$url = "https://$GH_USERNAME.github.io/$REPO_NAME/"
Write-Host ""
Write-Host "✅ 完成！" -ForegroundColor Green
Write-Host ""
Write-Host "你的流程图已上线：" -ForegroundColor Cyan
Write-Host "  $url" -ForegroundColor White
Write-Host ""
Write-Host "📱 用以下工具生成二维码：" -ForegroundColor Yellow
Write-Host "  - https://cli.im (草料二维码)"
Write-Host "  - https://qrcode.tecfuture.com"
Write-Host ""
Write-Host "提示：GitHub Pages 可能需要1-2分钟生效" -ForegroundColor Gray
