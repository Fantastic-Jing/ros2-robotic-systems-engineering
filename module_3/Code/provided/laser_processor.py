import math
import numpy as np
import logging

logger = logging.getLogger(__name__)

class LaserProcessor:
    """Utilities for the TurtleBot3 Laser Scanner."""
    
    @staticmethod
    def wrap(angle_rad):
        """Normalize angle(s) to [0, 2*pi)."""
        return np.mod(angle_rad, 2 * np.pi)

    def get_sector(
        self,
        scan,
        start_deg: float,
        end_deg: float,
        mode: str = "min"
    ):
        """
        Extract laser scan data in a sector and return:

            mode="min"      -> minimum distance
            mode="max"      -> maximum distance
            mode="average"  -> mean distance

        Angles are specified in degrees.
        """

        # Use the logger instance inside your methods
        logger.debug(f"Processing scan with {len(scan.ranges)} elements.")
        if not scan.ranges:
            logger.warning("Received an empty ranges list!")
            return None

        start_rad = self.wrap(math.radians(start_deg))
        end_rad = self.wrap(math.radians(end_deg))

        angle_min = scan.angle_min
        inc = scan.angle_increment

        values = np.asarray(scan.ranges)
        logger.debug(f"values array: {values}")

        # Compute angle for every scan point
        angles = self.wrap(
            angle_min + np.arange(values.size) * inc
        )
        # print with all digits: logger.debug(f"Content of angles array: {angles}")
        # print strings with two digits: logger.debug(f"Content of angles array: {[f'{val:.2f}' for val in angles]}")
        with np.printoptions(precision=2, suppress=True):
            logger.debug(f"angles array: {np.rad2deg(angles)}")

        # Determine sector mask
        if start_rad <= end_rad:
            mask = (angles >= start_rad) & (angles <= end_rad)
        else:
            # Sector crosses the 0° / 360° boundary
            mask = (angles >= start_rad) | (angles <= end_rad)

        sector_values = values[mask]

        # Keep only valid measurements
        sector_values = sector_values[
            np.isfinite(sector_values)
        ]

        sector_values = sector_values[
            (sector_values >= scan.range_min)
            & (sector_values <= scan.range_max)
        ]
        logger.debug(f"sector_values (valid and within range) array: {sector_values}")

        if sector_values.size == 0:
            return np.nan

        if mode == "min":
            return float(np.min(sector_values))

        elif mode == "max":
            return float(np.max(sector_values))

        elif mode in ("avg", "average", "mean"):
            return float(np.mean(sector_values))

        raise ValueError(
            f"Unknown mode '{mode}'. Use: min | max | average"
        )
    
    