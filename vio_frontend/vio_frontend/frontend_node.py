"""
VIO Frontend Node

Subscribes to camera images and performs:
1. Shi-Tomasi corner detection
2. Lucas-Kanade optical flow tracking with forward-backward check
3. RANSAC outlier rejection via essential matrix
4. Publishes tracked feature correspondences for the backend
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vio_interfaces.msg import TrackedFeatures
from cv_bridge import CvBridge
import cv2
import numpy as np


class VioFrontendNode(Node):
    def __init__(self):
        super().__init__('vio_frontend_node')

        # Parameters
        self.declare_parameter('max_features', 150)
        self.declare_parameter('min_features', 80)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('min_distance', 20.0)
        self.declare_parameter('lk_win_size', 21)
        self.declare_parameter('lk_max_level', 3)
        self.declare_parameter('ransac_threshold', 1.0)
        self.declare_parameter('fb_threshold', 1.0)
        self.declare_parameter('publish_debug_image', True)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.caminfo_callback, 10)

        # Publishers
        self.features_pub = self.create_publisher(
            TrackedFeatures, '/vio/tracked_features', 10)
        self.debug_img_pub = self.create_publisher(
            Image, '/vio/debug/image', 5)

        # State
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_pts = None
        self.cur_pts = None
        self.feature_ids = None
        self.track_lengths = None
        self.next_feature_id = 0
        self.frame_id = 0
        self.K = None
        self.dist_coeffs = None

        self.get_logger().info('VIO Frontend initialized')

    def caminfo_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            self.get_logger().info(
                f'Camera intrinsics received: fx={self.K[0,0]:.1f}, '
                f'fy={self.K[1,1]:.1f}, cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}')

    def image_callback(self, msg):
        if self.K is None:
            return

        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        if self.prev_gray is None:
            # First frame: detect features only
            self.prev_pts = self._detect_features(gray)
            n = len(self.prev_pts)
            self.feature_ids = np.arange(n, dtype=np.uint32)
            self.track_lengths = np.ones(n, dtype=np.uint32)
            self.next_feature_id = n
            self.prev_gray = gray
            self.frame_id += 1
            return

        # Track existing features
        if len(self.prev_pts) == 0:
            # Lost all features, re-detect
            self.prev_pts = self._detect_features(gray)
            n = len(self.prev_pts)
            self.feature_ids = np.arange(
                self.next_feature_id, self.next_feature_id + n, dtype=np.uint32)
            self.track_lengths = np.ones(n, dtype=np.uint32)
            self.next_feature_id += n
            self.prev_gray = gray
            self.frame_id += 1
            return

        cur_pts, good_mask = self._track_features(self.prev_gray, gray, self.prev_pts)
        prev_tracked = self.prev_pts[good_mask]
        cur_tracked = cur_pts
        ids_tracked = self.feature_ids[good_mask]
        lengths_tracked = self.track_lengths[good_mask] + 1

        # Outlier rejection with RANSAC
        if len(cur_tracked) >= 8:
            inlier_mask = self._reject_outliers(prev_tracked, cur_tracked)
            prev_tracked = prev_tracked[inlier_mask]
            cur_tracked = cur_tracked[inlier_mask]
            ids_tracked = ids_tracked[inlier_mask]
            lengths_tracked = lengths_tracked[inlier_mask]

        # Detect new features if below threshold
        min_feats = self.get_parameter('min_features').value
        if len(cur_tracked) < min_feats:
            new_pts, new_ids, new_lengths = self._detect_new_features(
                gray, cur_tracked)
            if len(new_pts) > 0:
                prev_for_new = new_pts.copy()  # no previous for new features
                prev_tracked = np.vstack([prev_tracked, prev_for_new])
                cur_tracked = np.vstack([cur_tracked, new_pts])
                ids_tracked = np.concatenate([ids_tracked, new_ids])
                lengths_tracked = np.concatenate([lengths_tracked, new_lengths])

        # Publish tracked features
        self._publish_features(msg.header, cur_tracked, prev_tracked,
                               ids_tracked, lengths_tracked)

        # Publish debug visualization
        if self.get_parameter('publish_debug_image').value:
            self._publish_debug_image(msg.header, gray, prev_tracked, cur_tracked,
                                       ids_tracked, lengths_tracked)

        # Update state for next frame
        self.prev_gray = gray
        self.prev_pts = cur_tracked
        self.feature_ids = ids_tracked
        self.track_lengths = lengths_tracked
        self.frame_id += 1

    def _detect_features(self, gray):
        """Detect Shi-Tomasi corners."""
        max_corners = self.get_parameter('max_features').value
        quality = self.get_parameter('quality_level').value
        min_dist = self.get_parameter('min_distance').value

        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=max_corners,
            qualityLevel=quality,
            minDistance=min_dist
        )
        if pts is None:
            return np.empty((0, 2), dtype=np.float32)
        return pts.reshape(-1, 2).astype(np.float32)

    def _track_features(self, prev_gray, cur_gray, prev_pts):
        """
        Lucas-Kanade optical flow with forward-backward consistency check.
        Returns (tracked_points, good_mask).
        """
        win = self.get_parameter('lk_win_size').value
        max_level = self.get_parameter('lk_max_level').value
        fb_thresh = self.get_parameter('fb_threshold').value

        lk_params = dict(
            winSize=(win, win),
            maxLevel=max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        pts_input = prev_pts.reshape(-1, 1, 2).astype(np.float32)

        # Forward track: prev -> cur
        cur_pts, status_fwd, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, cur_gray, pts_input, None, **lk_params)

        # Backward track: cur -> prev (for consistency check)
        rev_pts, status_bwd, _ = cv2.calcOpticalFlowPyrLK(
            cur_gray, prev_gray, cur_pts, None, **lk_params)

        cur_pts = cur_pts.reshape(-1, 2)
        rev_pts = rev_pts.reshape(-1, 2)

        # Forward-backward distance
        fb_dist = np.linalg.norm(prev_pts - rev_pts, axis=1)

        # Combined mask: both tracks succeeded + low FB distance + within image bounds
        h, w = cur_gray.shape
        good = (
            (status_fwd.flatten() == 1) &
            (status_bwd.flatten() == 1) &
            (fb_dist < fb_thresh) &
            (cur_pts[:, 0] >= 0) & (cur_pts[:, 0] < w) &
            (cur_pts[:, 1] >= 0) & (cur_pts[:, 1] < h)
        )

        return cur_pts[good], good

    def _reject_outliers(self, prev_pts, cur_pts):
        """RANSAC outlier rejection using the essential matrix."""
        thresh = self.get_parameter('ransac_threshold').value

        # Undistort to normalized coordinates
        prev_norm = cv2.undistortPoints(
            prev_pts.reshape(-1, 1, 2), self.K, self.dist_coeffs)
        cur_norm = cv2.undistortPoints(
            cur_pts.reshape(-1, 1, 2), self.K, self.dist_coeffs)

        # Find essential matrix with RANSAC
        E, mask = cv2.findEssentialMat(
            prev_norm, cur_norm,
            focal=1.0, pp=(0.0, 0.0),
            method=cv2.RANSAC,
            prob=0.999,
            threshold=thresh / self.K[0, 0]
        )

        if mask is None:
            return np.ones(len(cur_pts), dtype=bool)

        return mask.flatten().astype(bool)

    def _detect_new_features(self, gray, existing_pts):
        """Detect new features avoiding existing tracked regions."""
        max_feats = self.get_parameter('max_features').value
        quality = self.get_parameter('quality_level').value
        min_dist = int(self.get_parameter('min_distance').value)

        # Create mask to avoid detecting near existing features
        mask = np.ones(gray.shape, dtype=np.uint8) * 255
        for pt in existing_pts:
            cv2.circle(mask, (int(pt[0]), int(pt[1])), min_dist, 0, -1)

        max_new = max_feats - len(existing_pts)
        if max_new <= 0:
            return np.empty((0, 2), dtype=np.float32), \
                   np.array([], dtype=np.uint32), \
                   np.array([], dtype=np.uint32)

        new_pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=max_new,
            qualityLevel=quality,
            minDistance=min_dist,
            mask=mask
        )

        if new_pts is None:
            return np.empty((0, 2), dtype=np.float32), \
                   np.array([], dtype=np.uint32), \
                   np.array([], dtype=np.uint32)

        new_pts = new_pts.reshape(-1, 2).astype(np.float32)
        n = len(new_pts)
        new_ids = np.arange(
            self.next_feature_id, self.next_feature_id + n, dtype=np.uint32)
        self.next_feature_id += n
        new_lengths = np.ones(n, dtype=np.uint32)

        return new_pts, new_ids, new_lengths

    def _publish_features(self, header, cur_pts, prev_pts, feature_ids, track_lengths):
        """Publish tracked features message."""
        msg = TrackedFeatures()
        msg.header = header
        msg.frame_id = self.frame_id
        msg.num_features = len(cur_pts)
        msg.feature_ids = feature_ids.astype(np.uint32).tolist()
        msg.u_cur = cur_pts[:, 0].astype(np.float32).tolist()
        msg.v_cur = cur_pts[:, 1].astype(np.float32).tolist()
        msg.u_prev = prev_pts[:, 0].astype(np.float32).tolist()
        msg.v_prev = prev_pts[:, 1].astype(np.float32).tolist()
        msg.track_lengths = track_lengths.astype(np.uint32).tolist()
        self.features_pub.publish(msg)

    def _publish_debug_image(self, header, gray, prev_pts, cur_pts,
                              feature_ids, track_lengths):
        """Publish debug image with feature tracks drawn."""
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        for i in range(len(cur_pts)):
            length = int(track_lengths[i])
            # Color based on track length: short=red, long=green
            ratio = min(length / 20.0, 1.0)
            color = (0, int(255 * ratio), int(255 * (1.0 - ratio)))

            pt_cur = (int(cur_pts[i, 0]), int(cur_pts[i, 1]))
            pt_prev = (int(prev_pts[i, 0]), int(prev_pts[i, 1]))

            cv2.circle(debug, pt_cur, 3, color, -1)
            cv2.line(debug, pt_prev, pt_cur, color, 1)

        # Status text
        cv2.putText(debug, f'Features: {len(cur_pts)}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug, f'Frame: {self.frame_id}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        debug_msg.header = header
        self.debug_img_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VioFrontendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
