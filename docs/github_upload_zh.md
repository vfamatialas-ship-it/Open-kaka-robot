# 上传到 GitHub

## 方法一：GitHub Desktop

这是最适合第一次使用 GitHub 的方式。

1. 安装 GitHub Desktop。
2. 登录 GitHub 账号。
3. 点击 `File -> Add local repository`。
4. 选择 `open-kaka-V0` 文件夹。
5. 检查左侧 changed files，确认没有 `node_modules`、`dist`、`__pycache__`、日志、数据集。
6. 填写 commit message，例如：

```text
Initial Open Kaka V0 release
```

7. 点击 `Commit to main`。
8. 点击 `Publish repository`。

## 方法二：命令行

在 GitHub 上先创建一个空仓库，比如：

```text
open-kaka-v0
```

然后在 PowerShell 运行：

```powershell
cd "D:\Open kaka\contrl_soft\Bridge\openkaka_soft\open-kaka-V0"
git init
git add .
git commit -m "Initial Open Kaka V0 release"
git branch -M main
git remote add origin https://github.com/<your-name>/open-kaka-v0.git
git push -u origin main
```

如果电脑里没有 `git` 命令，请先安装 Git for Windows，并重新打开 PowerShell。

## 公开仓库前检查

上传前确认这些内容没有进入仓库：

```text
web_viewer/pink_slave_3d/node_modules/
web_viewer/pink_slave_3d/dist/
third_party/damiao/DM_Control_Python/
logs/
datasets/
__pycache__/
*.pyc
```

如果 GitHub 页面上看到这些内容，说明 `.gitignore` 没生效，需要删除后重新提交。
