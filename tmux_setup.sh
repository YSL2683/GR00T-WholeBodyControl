#!/bin/bash

SESSION_NAME="sonic"
WINDOW_NAME="main"
ROOT_DIR="${PROJECT_DIR:-${HOME}/GR00T-WholeBodyControl}"

if [ ! -d "$ROOT_DIR" ]; then
  echo "[경고] 디렉토리가 존재하지 않습니다: $ROOT_DIR"
  exit 1
fi

tmux has-session -t "$SESSION_NAME" 2>/dev/null

if [ $? -ne 0 ]; then
  # 1. 넉넉한 초기 크기(-x, -y)를 부여하여 세션 생성 (Pane 1개 확보)
  tmux new-session -d -s "$SESSION_NAME" -n "$WINDOW_NAME" -x 200 -y 50 -c "$ROOT_DIR"

  # 2. 페인을 단순히 3번 추가 분할 (총 4개 페인 생성)
  tmux split-window -t "$SESSION_NAME:$WINDOW_NAME" -c "$ROOT_DIR"
  tmux split-window -t "$SESSION_NAME:$WINDOW_NAME" -c "$ROOT_DIR"
  tmux split-window -t "$SESSION_NAME:$WINDOW_NAME" -c "$ROOT_DIR"

  # 3. 4개 페인을 2x2 격자 형태로 자동 재배치
  tmux select-layout -t "$SESSION_NAME:$WINDOW_NAME" tiled

  # 4. 첫 번째 페인으로 포커스 이동
  tmux select-pane -t "$SESSION_NAME:$WINDOW_NAME.0"
fi

if [ -n "$TMUX" ]; then
  tmux switch-client -t "$SESSION_NAME"
else
  tmux attach-session -t "$SESSION_NAME"
fi