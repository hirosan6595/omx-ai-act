"""俯瞰カメラの画像を、学習時のカメラ位置に合わせて補正して配信するノード。

/camera1/image_raw/compressed を購読し、基準画像(学習データの開始画像の中央値)との
相似変換(回転・移動・拡大)を推定して、その分だけ画像を変形し、
/camera1_aligned/image_raw/compressed に配信する。

起動時に /tmp/cam_align_state.json があれば、その変換を読み込んで使う(LOAD_STATE=0 で無効)。
無ければ、起動時(アームが初期姿勢で静止している前提)に複数フレームで推定し、固定する。
推定し直すときは、/tmp/cam_align_reestimate を作成する(touch)。
現在の変換は /tmp/cam_align_state.json に書き出す。
"""
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

REF = os.environ.get('REF', '/tmp/ref_wrist.png')
ROI = (100, 80, 620, 480)  # 机の範囲 (640x480 の基準画像の座標)
N_EST = 15                 # 推定に使うフレーム数
TRIGGER = '/tmp/cam_align_reestimate'
STATE = '/tmp/cam_align_state.json'


class CamAlign(Node):

    def __init__(self):
        super().__init__('cam_align')
        ref = cv2.resize(cv2.imread(REF), (640, 480))
        rg = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(rg)
        mask[ROI[1]:ROI[3], ROI[0]:ROI[2]] = 255
        self.orb = cv2.ORB_create(4000)
        self.kr, self.dr = self.orb.detectAndCompute(rg, mask)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.M = None          # 実機 -> 基準 の 2x3 行列
        self.samples = []
        self.pub = self.create_publisher(
            CompressedImage, '/camera1_aligned/image_raw/compressed', 10)
        self.create_subscription(
            CompressedImage, '/camera1/image_raw/compressed', self.cb, 10)
        if os.environ.get('LOAD_STATE', '1') != '0' and os.path.exists(STATE):
            st = json.load(open(STATE))
            self.M = np.array(st['matrix'], dtype=np.float64)
            self.get_logger().info('loaded saved transform (%s): rot %.2f deg, dx %.1f, dy %.1f, scale %.4f' % (
                st['time'], st['rot_deg'], st['dx'], st['dy'], st['scale']))
        else:
            self.get_logger().info('estimating transform from %d frames...' % N_EST)

    def estimate(self, gray):
        k, d = self.orb.detectAndCompute(gray, None)
        if d is None:
            return None
        m = sorted(self.bf.match(self.dr, d), key=lambda x: x.distance)[:500]
        if len(m) < 30:
            return None
        src = np.float32([k[x.trainIdx].pt for x in m])    # 実機
        dst = np.float32([self.kr[x.queryIdx].pt for x in m])  # 基準
        A, inl = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=3.0)
        if A is None or inl.sum() < 50:
            return None
        return A, int(inl.sum())

    def cb(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        if img.shape[:2] != (480, 640):
            img = cv2.resize(img, (640, 480))

        if os.path.exists(TRIGGER):
            os.remove(TRIGGER)
            self.M, self.samples = None, []
            self.get_logger().info('re-estimating transform...')

        if self.M is None:
            r = self.estimate(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            if r is not None:
                self.samples.append(r)
            if len(self.samples) >= N_EST:
                # 各成分の中央値を取る(外れ値に強い)
                self.M = np.median(np.stack([s[0] for s in self.samples]), axis=0)
                rot = np.degrees(np.arctan2(self.M[1, 0], self.M[0, 0]))
                scale = float(np.hypot(self.M[0, 0], self.M[1, 0]))
                c = self.M[:, :2] @ np.float32([320, 240]) + self.M[:, 2]
                state = {
                    'time': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'rot_deg': round(float(rot), 2),
                    'dx': round(float(c[0] - 320), 1),
                    'dy': round(float(c[1] - 240), 1),
                    'scale': round(scale, 4),
                    'inliers_median': int(np.median([s[1] for s in self.samples])),
                    'matrix': self.M.tolist(),
                }
                tmp = STATE + '.tmp'
                json.dump(state, open(tmp, 'w'), indent=1)
                os.replace(tmp, STATE)
                self.get_logger().info('transform fixed: %s' % json.dumps(
                    {k: v for k, v in state.items() if k != 'matrix'}))
            return  # 推定中は配信しない(未補正の画像を渡さないため)

        out = cv2.warpAffine(img, self.M, (640, 480),
                             flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        ok, enc = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            return
        o = CompressedImage()
        o.header = msg.header
        o.format = 'jpeg'
        o.data = enc.tobytes()
        self.pub.publish(o)


def main():
    rclpy.init()
    rclpy.spin(CamAlign())


if __name__ == '__main__':
    main()
