"""学習データ(各エピソードの開始画像)の中央値から、俯瞰カメラの基準画像を作る(コンテナ physical_ai_server 内で実行)。
出力: /tmp/ref_med.png -> cam_align/ref_wrist.png と tools/align/ref.png に置く。"""
import numpy as np,cv2
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds=LeRobotDataset("hirosan6595/omx_f_wrist-1028-train")
ep=ds.hf_dataset["episode_index"]; first={}
for i,e in enumerate(ep): first.setdefault(int(e),i)
ims=[]
for e in sorted(first):
    im=(ds[first[e]]["observation.images.camera1"].permute(1,2,0).numpy()*255).astype(np.uint8); ims.append(im)
A=np.stack(ims); med=np.median(A,axis=0).astype(np.uint8); print("shape",med.shape)
cv2.imwrite("/tmp/ref_med.png",med[...,::-1])
# camera drift during collection: mean abs diff to median for gray, by episode groups
g=A.astype(np.float32).mean(-1); gm=med.astype(np.float32).mean(-1)
d=np.abs(g-gm).mean((1,2)); print("mean|diff to median| first10 %.1f mid10 %.1f last10 %.1f"%(d[:10].mean(),d[32:42].mean(),d[-10:].mean()))
