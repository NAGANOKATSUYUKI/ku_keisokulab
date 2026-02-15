# Git 使い方 README

このドキュメントは、`/home/keisoku/catkin_ws/git` の内容をもとに、Gitの基本操作をまとめたものです。

## 1. 初期設定

```bash
git config --global user.name "あなたの名前"
git config --global user.email "あなたのメールアドレス"
git config --list
```

## 2. リポジトリの準備

### 新規で始める

```bash
mkdir my_project
cd my_project
git init
```

### 既存リポジトリを取得する

```bash
git clone <リポジトリURL>
cd <リポジトリ名>
```

## 3. 日常的な作業フロー

```bash
git status
git add .                       # または git add <ファイル名>
git commit -m "変更内容"
git pull origin master          # ブランチ名は環境に合わせて変更
git push origin master          # ブランチ名は環境に合わせて変更
```

## 4. 履歴の確認

```bash
git log --oneline --graph --decorate --all
git status
```

## 5. ブランチ操作

```bash
git branch <ブランチ名>
git checkout <ブランチ名>
```

新規作成と同時に切り替える場合:

```bash
git checkout -b <ブランチ名>
```

一覧表示と削除:

```bash
git branch
git branch -d <ブランチ名>      # マージ済みのみ削除
git branch -D <ブランチ名>      # 強制削除
```

## 6. リモート設定

```bash
git remote add origin <リポジトリURL>
git pull origin master
git push origin master
```

## 7. 変更の取り消し・修正

```bash
git checkout -- <ファイル名>     # ステージ前の変更を破棄
git reset HEAD <ファイル名>      # ステージ解除
git revert HEAD                  # 直前コミットを打ち消すコミットを作成
```

注意: 以下は履歴や作業内容を壊す可能性があるため、使う前に確認すること。

```bash
git reset --hard HEAD^
git push --force origin master
```

## 8. ファイル削除・タグ

```bash
git rm <ファイル名>
git tag <タグ名>
git tag -d <タグ名>
```

## 9. 便利な操作

```bash
git cherry-pick <コミットID>
git checkout <ブランチ名> -- <ファイル名>
```

## 10. ヘルプ

```bash
git help
git help commit
```

## 補足リンク

- GitHub SSH接続
  - https://codelikes.com/github-ssh-connection/
  - https://tech-reach.jp/column/1387/
- VSCodeとGit連携
  - https://miya-system-works.com/blog/detail/vscode-github/

