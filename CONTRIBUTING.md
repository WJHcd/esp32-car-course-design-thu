# 多人协作 Git 工作流（必读）

> 适用于本课程设计小组的轻量协作规范。目标是：**main 永远可构建、可烧录，改动都有记录、可回退**。
> 全文约 3 分钟可读完，建议每位组员第一次提交前通读一遍。

---

## 0. 一页速览（先看这个）

```
① 拉最新代码         git pull
② 开功能分支         git checkout -b feature/xxx
③ 写代码 + 本地提交   git add . && git commit -m "feat: ..."
④ 推送到远程         git push -u origin feature/xxx
⑤ 在 GitHub 上提 PR  网页操作（创建 Pull Request）
⑥ 合并到 main        至少 1 人 review 后 squash 合并
⑦ 回到 main 更新      git checkout main && git pull
```

**三条铁律：**
1. **不要在 main 上直接写代码**，所有改动走分支 + PR。
2. **不要提交 `build/`、`.venv/`、`managed_components/`**（已在 `.gitignore` 里，但不要手动 `git add -f`）。
3. **每天开始工作前先 `git pull`**，避免积压大量冲突。

---

## 1. 分支模型（轻量 GitHub Flow）

只保留两类分支，简单够用：

| 分支 | 用途 | 谁可以推 |
| --- | --- | --- |
| `main` | 稳定分支，始终可构建可烧录 | 只能通过 PR 合并 |
| `feature/xxx` | 每个功能/任务一条分支 | 对应开发者 |

### 分支命名规则

| 类型 | 格式 | 示例 |
| --- | --- | --- |
| 新功能 | `feature/<功能名>` | `feature/tracking-line` |
| 修 bug | `fix/<问题>` | `fix/encoder-glitch` |
| 文档 | `docs/<主题>` | `docs/pin-map` |
| 实验/尝试 | `experiment/<内容>` | `experiment/pid-tuning` |

> 分支名用英文小写 + 连字符，避免中文/空格在命令行和网页端的各种坑。

---

## 2. Commit 规范

### 2.1 提交信息格式

```
<type>: <简短描述>

（可选）详细说明：为什么改、怎么改、影响什么
```

### 2.2 type 一览（选一个最贴切的）

| type | 含义 |
| --- | --- |
| `feat` | 新功能 |
| `fix` | 修 bug |
| `docs` | 文档改动 |
| `refactor` | 重构，不改功能 |
| `test` | 加/改测试 |
| `chore` | 杂项（改配置、依赖等） |

### 2.3 示例

```
feat: 增加三路编码器 PCNT 读取

- 使用 ESP32-S3 PCNT 单元 0/1/2 读取 E1/E2/E3
- 频率 20kHz，滤波消除毛刺
- 接线见 docs/motor-drive-test/接线说明.md
```

```
fix: 修正电机 B 方向逻辑反接问题
```

```
docs: 补充 D24A 驱动板引脚说明
```

> 建议：描述用中文即可，团队一致最重要；**一个逻辑改动一个 commit**，不要攒一堆再一次性提交。

---

## 3. 标准开发流程（详细版）

### 3.1 开始一个新任务

```bash
# 1. 确保 main 是最新的
git checkout main
git pull

# 2. 从最新的 main 开分支
git checkout -b feature/tracking-line
```

### 3.2 开发过程中

```bash
# 查看改了什么（养成习惯）
git status
git diff

# 提交（分步 add 你真正想提交的文件，不要无脑 git add -A）
git add main/tracking.c main/tracking.h
git commit -m "feat: 红外循迹基本逻辑"
```

### 3.3 推送到 GitHub 并发起 PR

```bash
git push -u origin feature/tracking-line
```

然后到 GitHub 仓库页面 → **Pull requests → New pull request**，选择 `base: main ← compare: feature/tracking-line`，写清楚改了什么、测试情况，创建 PR。

### 3.4 合并

- 至少 1 名组员 review 通过后合并；
- 合并方式选 **Squash and merge**（把分支上多个 commit 压成一个），main 历史保持干净。

### 3.5 合并后清理

```bash
git checkout main
git pull                 # 拿到合并后的代码
git branch -d feature/tracking-line   # 删除本地分支（可选）
git push origin --delete feature/tracking-line   # 删除远程分支（可选）
```

---

## 4. 冲突处理

多人同时改同一文件时会出现冲突，**别慌，是正常现象**：

```bash
git pull          # 提示冲突
git status        # 查看冲突文件（显示 both modified）
```

打开冲突文件，`<<<<<<<` 和 `>>>>>>>` 之间就是要你决定的内容：

```
<<<<<<< HEAD
你的版本
=======
别人的版本
>>>>>>> feature/xxx
```

**处理方法：** 保留需要的部分，删掉 `<<<<<<<` / `=======` / `>>>>>>>` 标记，然后：

```bash
git add <冲突文件>
git commit -m "merge: 解决 XXX 冲突"
```

> 提示：如果改动涉及**引脚分配、电机编号、总线协议**这类公共定义（如 `motor.h`、`main.c` 的初始化部分），先和队友在群里沟通谁负责哪部分，从源头减少冲突。

---

## 5. 重要注意事项（踩坑预防）

### 5.1 ⚠️ 不要把仓库放在 OneDrive / 网盘同步目录里

本仓库当前位于 `OneDrive\...` 下，**Git 的 `.git` 目录与 OneDrive 同步机制冲突**，可能导致：
- 提交历史损坏、索引丢失；
- 多台设备同步出脏数据。

**强烈建议**：把仓库克隆到本地非云同步路径（如 `C:\esp-projects\`），用 GitHub 作为唯一协作中心。

### 5.2 ESP-IDF 项目注意事项

- `build/`、`managed_components/`、`dependencies.lock` 不入库（`.gitignore` 已配置）；
- 克隆后第一次构建前运行：`idf.py reconfigure`（会自动拉取 `managed_components` 依赖）；
- `sdkconfig` 会入库（保证构建可复现），改配置用 `idf.py menuconfig` 后记得提交。

### 5.3 换行符

`.gitattributes` 已统一为 LF 存储。如果 `git status` 莫名显示整个文件被改动，多半是换行符问题，不要提交，先跑：

```bash
git add --renormalize .
```

### 5.4 不要提交的内容

| 内容 | 原因 |
| --- | --- |
| `build/`、`managed_components/` | 可重新生成 |
| `.venv/` | 本机 Python 环境 |
| `.vscode/` | 个人 IDE 配置 |
| 串口/调试日志 | 无意义噪音 |

> 如果误提交了大文件，参考 GitHub 官方文档使用 `git filter-repo` 清理历史（需谨慎，会改写历史）。

---

## 6. 权限与分工建议

- **仓库管理员（1 人）**：负责 GitHub 仓库管理、把组员加为 Collaborator（Settings → Collaborators）、合并重要 PR；
- **建议在 GitHub 上开启 main 分支保护**：Settings → Branches → Add rule → 勾选 *Require a pull request before merging*（至少 1 人 review）；
- 每位组员有自己固定的分工模块（如 A 管电机驱动、B 管循迹避障、C 管摄像头），PR 互相 review，保证至少两人看过关键代码。

---

## 7. 常用命令速查

| 操作 | 命令 |
| --- | --- |
| 查看状态 | `git status` |
| 查看差异 | `git diff` |
| 查看历史 | `git log --oneline --graph` |
| 放弃未提交改动 | `git checkout -- <文件>` |
| 撤销上次 commit（保留改动） | `git reset --soft HEAD~1` |
| 暂存当前改动 | `git stash` / `git stash pop` |
| 看远程分支 | `git branch -r` |
