#!/bin/bash
# 俯瞰カメラ補正ノード(cam_align_node.py)を open_manipulator コンテナ内で動かし続ける。
# - コンテナが動いていなければ待つ。コンテナが再起動したら、ノードも起動し直す。
# - 推定した変換(state.json)をホストに保存し、次回の起動で読み込む。
D="$(cd "$(dirname "$0")" && pwd)"
C=open_manipulator
ROS='source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp'
log() { echo "$(date '+%F %T') $*"; }
while true; do
  if [ "$(docker inspect -f '{{.State.Running}}' $C 2>/dev/null)" = "true" ]; then
    docker cp "$D/cam_align_node.py" $C:/tmp/cam_align_node.py
    docker cp "$D/ref_wrist.png" $C:/tmp/ref_wrist.png
    if [ -f "$D/state.json" ]; then docker cp "$D/state.json" $C:/tmp/cam_align_state.json; fi
    # docker cp したファイルはホストのユーザー所有になる。/tmp は保護されていて root でも上書きできないので、所有者を root にする
    docker exec $C bash -c 'chown root:root /tmp/cam_align_node.py /tmp/ref_wrist.png /tmp/cam_align_state.json 2>/dev/null; true'
    # 古いノードを止める(終了の合図で止まらなければ強制終了)
    docker exec $C bash -c 'pkill -f "^python3 /tmp/cam_align_node.py"; sleep 2; pkill -9 -f "^python3 /tmp/cam_align_node.py"; true'
    log "starting cam_align_node"
    docker exec $C bash -c "$ROS; exec python3 /tmp/cam_align_node.py" &
    PID=$!
    while kill -0 $PID 2>/dev/null; do
      sleep 10
      # 推定し直した変換を、ホストへ保存
      if docker cp $C:/tmp/cam_align_state.json "$D/state.json.tmp" 2>/dev/null; then
        cmp -s "$D/state.json.tmp" "$D/state.json" || { mv "$D/state.json.tmp" "$D/state.json"; log "saved new transform"; }
        rm -f "$D/state.json.tmp"
      fi
    done
    log "cam_align_node exited; restarting"
  fi
  sleep 5
done
