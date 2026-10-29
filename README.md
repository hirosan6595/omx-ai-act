# OMX-AI ACT 模倣学習：成功率を上げるための追加分

ROBOTIS OMX-AI（フォロワー OMX-F／リーダー OMX-L）で、ACT（physical_ai_tools / lerobot）に
「キューブをつかんで箱に入れる」動作を学習させたときの、**ROBOTIS のコードに加えた変更と、追加したコード**です。
経過と結果の詳細は [report/README.md](report/README.md) にあります。

## 結果

| 条件（030000 ステップ、n_action_steps=100） | 成功率 |
|---|---|
| broadcaster 有効（ROBOTIS の既定のまま） | 0/61（ログ集計） |
| broadcaster 無効 ＋ カメラを手で合わせる | 11/15（73%） |
| broadcaster 無効 ＋ **カメラのソフト補正** | **14/15（93%）** |

## 本質的な変更は 2 つ

### 1. 推論中は broadcaster を止める（いちばん効いた）
`omx_ai.launch` は、リーダーの姿勢を `/leader/joint_trajectory` に 100Hz で送る
`joint_trajectory_command_broadcaster` を起動します。推論も同じトピックに 30Hz で指令を送るため、
**グリッパーの指令がぶつかり**（リーダー側は「開」≒0.65）、グリッパーが閉じきれずに震えて、キューブを保持できません。

```bash
# ホストで（推論の前）
tools/broadcaster.sh off
# 録画の前は、omx_ai.launch を再起動して元に戻す（または tools/broadcaster.sh on）
```

### 2. 俯瞰カメラのずれをソフトで補正する
カメラの取り付けが少しずつずれ、学習時の画像と合わなくなると成功率が下がります。
[`cam_align/cam_align_node.py`](cam_align/cam_align_node.py) は、基準画像（学習データの開始画像の中央値）との
相似変換（回転・移動・拡大）を ORB＋RANSAC で求め、画像を変形して `/camera1_aligned/image_raw/compressed` に配信します。
推論側は、この補正後の画像を使います（[patches/physical_ai_tools.patch](patches/physical_ai_tools.patch)）。

## 構成

| パス | 分類 | 内容 |
|---|---|---|
| `cam_align/cam_align_node.py` | 本質 | カメラ補正ノード（コンテナ open_manipulator 内で動く） |
| `cam_align/ref_wrist.png` | 本質 | 補正の基準画像（`tools/mkref.py` で作成） |
| `patches/physical_ai_tools.patch` | 本質 | 推論の入力を補正後の画像にし、手首カメラ（camera2）も使う |
| `tools/broadcaster.sh` | 本質 | broadcaster を止める／動かす |
| `cam_align/run.sh`, `cam-align.service` | 運用 | 補正ノードを自動起動し、推定した変換を `state.json` に保存 |
| `cam_align/reestimate.sh` | 運用 | カメラが動いたときに推定し直す（アームを初期姿勢にしてから） |
| `patches/open_manipulator.patch` | 運用 | シリアルポートを by-id で固定（OMX-F／OMX-L の取り違え防止） |
| `tools/align_loop2.py`, `tools/align/` | 運用 | 手でカメラを合わせるための画面（5 か所のずれを表示） |
| `tools/gohome.py` | 運用 | アームを初期姿勢へ戻す |
| `tools/mkref.py` | 運用 | 学習データから基準画像を作る |
| `report/` | 記録 | 経過・結果・考察のレポート |

## 元にしたコード（このリポジトリには含めていません）

| リポジトリ | コミット |
|---|---|
| [ROBOTIS-GIT/physical_ai_tools](https://github.com/ROBOTIS-GIT/physical_ai_tools) | `27192daf75fa5f421a3bd1825a50ee63d649dccd` |
| [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) | `ee9a720bcc45acda9c0df8fe689b1182844307db` |

```bash
git -C physical_ai_tools checkout 27192daf && git -C physical_ai_tools apply ../omx-ai-act/patches/physical_ai_tools.patch
git -C open_manipulator  checkout ee9a720b && git -C open_manipulator  apply ../omx-ai-act/patches/open_manipulator.patch
```
`open_manipulator.patch` のシリアル番号は `<FOLLOWER_SERIAL>` / `<LEADER_SERIAL>` にしてあります。`ls /dev/serial/by-id/` で自分の機体の番号を調べて書き換えてから当ててください。

## 使い方（ホストで）

```bash
# 補正ノードの自動起動（ExecStart のパスは ~/omx/cam_align を想定。置き場所に合わせて直す）
cp -r cam_align ~/omx/
cp cam_align/cam-align.service ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now cam-align.service

# 推論の前に毎回
docker exec open_manipulator bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_zenoh_cpp; python3 /path/to/gohome.py"
tools/broadcaster.sh off
~/omx/cam_align/reestimate.sh      # カメラが動いていそうなとき
```

手でカメラを合わせる画面：`tools/align_loop2.py` を動かし、`tools/` を Web サーバーで公開して `align/align2.html` を開きます
（カメラ画像は web_video_server の `localhost:8080` から取ります）。

## 試したが採用しなかったもの
- temporal ensemble（n_action_steps=1）：動きは滑らかだが中央でつかめない
- n_action_steps=50／25：運ぶ途中で止まる
- 射影変換での位置合わせ：机の中央に特徴点が少なく、推定が揺れる
