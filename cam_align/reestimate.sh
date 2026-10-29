#!/bin/bash
# カメラを動かしたときに、変換を推定し直す。先にアームを初期姿勢で静止させること。
docker exec open_manipulator touch /tmp/cam_align_reestimate && echo "推定し直します(約1秒)。結果: ~/omx/cam_align/state.json(10秒以内に更新)"
