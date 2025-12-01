# fusion.py
from __future__ import annotations
from dataclasses import dataclass
from collections import deque
from typing import Deque, Dict, Optional, Tuple, Any
import time
import math

Vec2 = Tuple[float, float]

IMU_ID_TO_FINGER  = {2:"thumb", 1:"index", 0:"middle", 7:"ring", 6:"pinky"}
LED_GPIO_TO_FINGER = {1:"thumb", 3:"index", 20:"middle", 7:"ring", 6:"pinky"}

# --- Clock sync: map device timestamp_us -> host monotonic seconds (per channel) ---
class DeviceClockSync:
    """ Simple affine mapping using first-seen pair."""
    def __init__(self) -> None:
        self._t0_host: Optional[float] = None
        self._t0_dev_us: Optional[int] = None

    def dev_to_host(self, host_rx_t: float, dev_us: int) -> float:
        """
        Uses first-seen pair (host_rx_t, dev_us) to create an affine mapping.
        Good enough if device clock drift is small over short windows.
        """
        if self._t0_host is None or self._t0_dev_us is None:
            self._t0_host = host_rx_t
            self._t0_dev_us = dev_us
            return host_rx_t
        return self._t0_host + (dev_us - self._t0_dev_us) / 1_000_000.0


@dataclass
class IMUSample:
    """ Single IMU sample at time t. """
    t: float
    # store whichever you have available; start with raw->(gx,gy,gz, ax,ay,az, mx,my,mz) in SI units
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]
    mag: Tuple[float, float, float]


@dataclass
class FingerCV:
    """ Latest CV centroid for a finger. """
    t: float
    centroid: Optional[Vec2]


@dataclass
class FingerIMU:
    """ Single IMU sample for a finger. """
    t: float
    # you can replace these with quaternion/ypr later
    sample: IMUSample


class FusionState:
    """ Fuses CV centroids and IMU samples per finger. """
    def __init__(self, imu_max_seconds: float = 1.5, cv_max_age_s: float = 0.25) -> None:
        self.cv_max_age_s = cv_max_age_s

        # latest CV centroid per finger (updated as LEDs cycle)
        self.cv_latest: Dict[str, FingerCV] = {f: FingerCV(t=-1.0, centroid=None)
                                               for f in IMU_ID_TO_FINGER.values()}

        # IMU ring buffers per finger
        self.imu_buf: Dict[str, Deque[FingerIMU]] = {f: deque() for f in IMU_ID_TO_FINGER.values()}
        self.imu_max_seconds = imu_max_seconds

        # if you want device->host sync per channel
        self.clock_sync: Dict[int, DeviceClockSync] = {ch: DeviceClockSync() for ch in IMU_ID_TO_FINGER.keys()}

    # ---------- Update CV ----------
    def update_cv(self, cv_packet: Dict[str, Any]) -> None:
        """ Update latest CV centroid for given LED index. """
        t = float(cv_packet["cv_timestamp"])
        led = int(cv_packet["led_index"])
        finger = LED_GPIO_TO_FINGER.get(led)
        if finger is None:
            return

        blob_centers = cv_packet.get("blob_centers", [])
        if not blob_centers:
            # centroid missing this frame for that finger
            self.cv_latest[finger] = FingerCV(t=t, centroid=None)
            return

        # If multiple blobs, pick the one closest to last centroid for that finger (simple, works well)
        prev = self.cv_latest[finger].centroid
        if prev is None:
            chosen = blob_centers[0]
        else:
            px, py = prev
            chosen = min(blob_centers, key=lambda c: (c[0]-px)**2 + (c[1]-py)**2)

        self.cv_latest[finger] = FingerCV(t=t, centroid=(float(chosen[0]), float(chosen[1])))

    # ---------- Update IMU ----------
    def update_imu(self, imu_packet: Dict[str, Any], host_rx_t: Optional[float] = None) -> None:
        """
        host_rx_t = time.monotonic() at BLE notification receipt.
        If not provided, we’ll stamp with time.monotonic() now.
        """
        host_rx_t = time.monotonic() if host_rx_t is None else host_rx_t

        ch = int(imu_packet["channel"])
        finger = IMU_ID_TO_FINGER.get(ch)
        if finger is None:
            return

        dev_us = int(imu_packet.get("timestamp_us", 0))
        # map device time -> host time for better alignment than raw receipt time
        t = self.clock_sync[ch].dev_to_host(host_rx_t, dev_us) if dev_us else host_rx_t

        ax, ay, az = imu_packet["accel_raw"]
        gx, gy, gz = imu_packet["gyro_raw"]
        mx, my, mz = imu_packet["mag_raw"]

        # TODO: convert raw -> SI using YOUR converter.py scaling
        # For now, treat as “raw units”; letter recognition will still work after you normalize.
        sample = IMUSample(
            t=t,
            gyro=(float(gx), float(gy), float(gz)),
            accel=(float(ax), float(ay), float(az)),
            mag=(float(mx), float(my), float(mz)),
        )

        buf = self.imu_buf[finger]
        buf.append(FingerIMU(t=t, sample=sample))

        tmin = t - self.imu_max_seconds
        while buf and buf[0].t < tmin:
            buf.popleft()

    # ---------- Query fused snapshot ----------
    def snapshot(self, t: Optional[float] = None) -> Dict[str, Any]:
        """
        Returns a single fused dict with latest centroid per finger + nearest IMU sample per finger.
        """
        t = time.monotonic() if t is None else t

        out: Dict[str, Any] = {"t": t, "fingers": {}}

        for finger in self.cv_latest.keys():
            # CV
            cv = self.cv_latest[finger]
            cv_ok = (cv.centroid is not None) and (t - cv.t <= self.cv_max_age_s)

            # IMU nearest
            imu_sample = self._nearest_imu(finger, t)

            out["fingers"][finger] = {
                "cv_centroid": cv.centroid if cv_ok else None,
                "cv_age_s": (t - cv.t) if cv.t >= 0 else None,
                "imu": imu_sample,  # raw for now; later replace with ypr/quaternion/bend
            }

        return out

    def _nearest_imu(self, finger: str, t: float) -> Optional[Dict[str, Any]]:
        """ Find nearest IMU sample to time t for given finger. """
        buf = self.imu_buf[finger]
        if not buf:
            return None
        best = min(buf, key=lambda s: abs(s.t - t))
        if abs(best.t - t) > 0.10:  # reject if too far (100ms)
            return None
        return {
            "t": best.t,
            "gyro": best.sample.gyro,
            "accel": best.sample.accel,
            "mag": best.sample.mag,
        }