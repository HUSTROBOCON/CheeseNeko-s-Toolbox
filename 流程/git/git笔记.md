# git笔记

## **一些用过的方法**

文件名包含中文，Git 默认用八进制编码显示（如 `\346\250...`），这让文件名很难阅读。

设置 Git 以正确显示中文文件名

`git config --global core.quotepath false; git status`

## **git commit 提交更改**

*   **标准提交**（需先 `git add`）：
    ```bash
    git commit -m "提交信息"
    ```

*   **自动暂存并提交**（仅限已跟踪文件，跳过 `git add` 步骤）：
    ```bash
    git commit -am "提交信息"
    ```

*   **修改最后一次提交**（用于修正漏交文件或改写提交信息）：
    ```bash
    git commit --amend -m "新的提交信息"
    ```

## **git push 推送更改**

*   **默认推送**（需已设定上游分支）：
    ```bash
    git push
    ```

*   **指定仓库和分支推送**：
    ```bash
    git push origin main
    ```

*   **初次推送并关联远程分支**（设置 Upstream）：
    ```bash
    git push -u origin main
    ```
    *   `-u` 选项用于将本地分支与远程分支关联，之后只需输入 `git push` 即可。





