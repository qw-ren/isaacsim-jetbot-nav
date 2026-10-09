# 四人协作简明指南

## 共同规则

- 使用一个主仓库，获得 Collaborator 权限后直接参与，**不用 Fork**。
- `main` 保持稳定、可运行。新任务使用 `feature/...`，修复使用 `fix/...`；按任务命名分支。
- **不要直接 push main**。正式修改统一经过分支 → commit → push → Pull Request（PR）→ review → 合并 main。
- PR 尽量小，写清改了什么、如何运行和验证。
- 上述是团队约定，是否强制执行取决于后续仓库分支保护设置。

## 1. 第一次下载

安装 Git，并完成 GitHub 登录。公开仓库可直接下载；上传分支与创建同仓库 PR 需要 Collaborator 写入权限。

```bash
git clone https://github.com/qw-ren/isaacsim-jetbot-nav.git
cd isaacsim-jetbot-nav
```

## 2. 开始新任务

先保存当前修改，确认工作区干净，再同步 main 并新建任务分支：

```bash
git switch main
git pull --ff-only origin main
git switch -c feature/nav2-baseline
```

其他示例：`feature/isaac-jetbot`、`feature/yolo-detection`、`feature/ppo-local-planner`、`fix/tf-tree`。后续命令中的分支名要换成自己的实际分支名。

## 3. 保存并上传

```bash
git status
git add <本次修改的文件或目录>
git diff --cached
git commit -m "feat: add Nav2 baseline configuration"
git push -u origin feature/nav2-baseline
```

`<本次修改的文件或目录>` 是占位说明，使用时替换，例如 `git add docs/README.md`。提交前查看暂存内容，避免误传大文件或敏感信息。

## 4. 创建 PR 并合并

在 GitHub 打开仓库，点击 **Compare & pull request**（或 Pull requests → New pull request），选择 `base: main`、`compare: 自己的分支`。写清修改内容、运行/验证方式、已知问题。检查通过后 Merge，可删除已合并分支。

下一项任务重新执行第 2 步，从最新 main 建立新分支。遇到合并冲突时逐个确认应保留的内容；不确定就和相关队友一起处理，不要强制覆盖 main。

## 不提交的内容

- 大模型权重（例如 `.pt`、`.pth`、`.onnx`、`.safetensors`）。
- 数据集、ROS bag、完整训练日志与视频输出。
- 大型 USD 场景、纹理及 Isaac Sim 缓存。
- 训练 checkpoint、缓存、虚拟环境和 ROS 2 构建产物。
- 密码、Token、密钥及个人环境配置。

这些文件放外部共享存储，仓库保留下载地址、版本和使用说明。小型必要资源可讨论后提交。`.gitignore` 无法移除已被 Git 跟踪的文件；若误提交，先停止上传并联系队友处理。
