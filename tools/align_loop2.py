import cv2,numpy as np,json,time,os,urllib.request,collections
# 机の範囲だけの特徴点で相似変換(回転・移動・拡大)を求め、キューブ5か所のずれ(px)で判定する
# (射影変換は、机の中央に特徴点が少なく推定が大きく揺れたため不採用)
D=os.path.dirname(os.path.abspath(__file__))+"/align/"
TOL_PX=float(os.environ.get('TOL_PX','5'))
ROI=(100,80,620,480)  # x0,y0,x1,y1 (640x480, 基準画像の座標)
PTS={"中央":(337,273),"左":(241,285),"右":(440,277),"奥":(340,246),"手前":(343,329)}
ref=cv2.imread(D+"ref.png"); rg=cv2.cvtColor(cv2.resize(ref,(640,480)),cv2.COLOR_BGR2GRAY)
mask=np.zeros_like(rg); mask[ROI[1]:ROI[3],ROI[0]:ROI[2]]=255
orb=cv2.ORB_create(4000); kr,dr=orb.detectAndCompute(rg,mask); bf=cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
H=collections.deque(maxlen=8); Q=np.float32(list(PTS.values())).reshape(-1,1,2)
while True:
    out={"updated":time.strftime("%H:%M:%S"),"tol_px":TOL_PX,"roi":ROI}
    try:
        b=urllib.request.urlopen("http://localhost:8080/snapshot?topic=/camera1/image_raw",timeout=6).read()
        g=cv2.cvtColor(cv2.resize(cv2.imdecode(np.frombuffer(b,np.uint8),1),(640,480)),cv2.COLOR_BGR2GRAY)
        k,d=orb.detectAndCompute(g,None); m=sorted(bf.match(dr,d),key=lambda x:x.distance)[:500]
        src=np.float32([kr[x.queryIdx].pt for x in m]); dst=np.float32([k[x.trainIdx].pt for x in m])
        S,inl=cv2.estimateAffinePartial2D(src,dst,method=cv2.RANSAC,ransacReprojThreshold=3.0)   # 基準 -> 実機
        if S is None: raise RuntimeError("no transform")
        ni=int(inl.sum())
        if ni<20: raise RuntimeError("too few matches (%d)"%ni)
        q=Q.reshape(-1,2); disp=(S[:,:2]@q.T).T+S[:,2]-q   # 同じ物理位置の、画像上の移動
        A,_=cv2.estimateAffinePartial2D(dst[inl.ravel()>0],src[inl.ravel()>0])        # 実機 -> 基準(方向の案内用)
        c=(A[:,:2]@np.float32([[320],[240]])).ravel()+A[:,2]
        H.append(np.r_[disp.ravel(),np.degrees(np.arctan2(A[1,0],A[0,0])),c[0]-320,c[1]-240,np.hypot(A[0,0],A[1,0])])
        v=np.median(np.array(H),0); dv=v[:2*len(PTS)].reshape(-1,2); dist=np.hypot(dv[:,0],dv[:,1])
        out.update(points=[{"name":n,"x":p[0],"y":p[1],"dx":round(float(a),1),"dy":round(float(b2),1),"dist":round(float(s),1)} for (n,p),(a,b2),s in zip(PTS.items(),dv,dist)],
                   max_px=round(float(dist.max()),1),ok=bool(dist.max()<=TOL_PX),inliers=ni,n_avg=len(H),
                   rot=round(float(v[-4]),2),dx=round(float(v[-3]),1),dy=round(float(v[-2]),1),scale=round(float(v[-1]),3))
    except Exception as e: out["error"]=str(e)
    json.dump(out,open(D+"align2.json.tmp","w"),ensure_ascii=False); os.replace(D+"align2.json.tmp",D+"align2.json"); time.sleep(0.7)
