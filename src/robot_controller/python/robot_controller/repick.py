"""Controller-owned physical-attempt history; no motion, clocks or disk persistence."""

from dataclasses import replace
import math
import threading

import numpy as np

from item_perception_yolo.pick_planning import (
    camera_body_fits, camera_body_footprint, rigid_matrix)
from item_perception_yolo.repick import MAX_REGIONS, blocking_region, contains, rejection_reason
from item_perception_yolo.surface_guard import HeightPlane


class TrickleTargetRejected(ValueError):
    """A valid configuration produced an unsafe adjusted endpoint; skip that pose."""


def trickle_plan(plan, configuration, extra_mm):
    """Lower only nominal final pick Z; validate the actual endpoint without clamping."""
    if not math.isfinite(extra_mm) or extra_mm < 0:
        raise ValueError("Trickle descent must be finite and nonnegative")
    matrix = rigid_matrix(plan[3].matrix, "Nominal pick target").copy()
    matrix[2, 3] -= extra_mm / 1000.
    selection = configuration.selection
    platform = selection.station.platform.base_from_platform
    height = HeightPlane(platform, "bin floor").measure(matrix[:3, 3])
    if height.below_plane:
        raise TrickleTargetRejected(height.reason.replace(
            "Measured surface", "Trickle pick target").replace("surface Z=", "target Z="))
    camera = (np.linalg.inv(platform) @ matrix
              @ rigid_matrix(selection.robot_camera.reference_from_camera_link,
                             "Robot camera mounting"))
    if not camera_body_fits(camera_body_footprint(camera),
                            [[p.x_m, p.y_m] for p in selection.bin.points]):
        raise TrickleTargetRejected("Trickle pick target moves robot-camera body outside Bin ROI")
    return (*plan[:3], replace(plan[3], matrix=matrix), *plan[4:]), height


class RepickHistory:
    def __init__(self, settings, events):
        self.settings = dict(settings)
        self.events = events
        self.lock = threading.RLock()
        self.regions = []
        self.revision = 0
        self.serial = 0

    def _event(self, kind, message, **details):
        self.events.record("INFO", "repick_" + kind, message, revision=self.revision, **details)

    def snapshot(self):
        with self.lock:
            return self.revision, [dict(r) for r in self.regions if r["cooldown_remaining"]]

    def blocked(self, position):
        return blocking_region(self.snapshot()[1], position)

    def prior_attempts(self, position, *, clearing_area_id=None):
        """Use the same oldest fixed circle as completed(), without changing history.

        A placement bridge may plan past its confirmed-release boundary: that
        next pick must treat only the old item's area as cleared. Actual history
        is still cleared exclusively by the PLACED callback.
        """
        with self.lock:
            region = next((r for r in self.regions
                           if r["id"] != clearing_area_id and contains(r, position)), None)
            return (region["attempts"], region["id"]) if region else (0, None)

    def completed(self, position, candidate_id):
        """Called once per confirmed HELD/FAILED outcome, including after Resume."""
        if not self.settings["blaclist_trigger"]:
            return None
        with self.lock:
            # Only attempts elsewhere age an already-active region. The newly
            # triggered region is added afterwards and cannot age itself.
            for region in tuple(self.regions):
                if region["cooldown_remaining"] and not contains(region, position):
                    region["cooldown_remaining"] -= 1
                    self.revision += 1
                    self._event("cooldown", "Repick cooldown advanced", region=dict(region))
                    if not region["cooldown_remaining"]:
                        self.regions.remove(region)
                        self._event("expired", "Repick area expired; count reset", region=region)
            region = next((r for r in self.regions if contains(r, position)), None)
            if region is None:
                if len(self.regions) == MAX_REGIONS:
                    # At most cooldown active regions can survive other completed
                    # attempts. Retire the oldest inactive observation at capacity.
                    retired = next(r for r in self.regions if not r["cooldown_remaining"])
                    self.regions.remove(retired)
                    self._event("retired", "Old inactive repick history retired", region=retired)
                self.serial += 1
                region = {"id": self.serial, "x_m": float(position[0]),
                          "y_m": float(position[1]),
                          "radius_m": self.settings["blackist_radius"] / 1000.,
                          "attempts": 0, "cooldown_remaining": 0}
                self.regions.append(region)
            region["attempts"] += 1
            if region["attempts"] >= self.settings["blaclist_trigger"]:
                region["cooldown_remaining"] = self.settings["blackist_cooldown"]
            self.revision += 1
            self._event("activated" if region["cooldown_remaining"] else "attempt",
                        "Physical pickup attempt recorded", candidate_id=candidate_id,
                        region=dict(region))
            return region["id"]

    def placed(self, area_id):
        with self.lock:
            region = next((r for r in self.regions if r["id"] == area_id), None)
            if region is not None:
                self.regions.remove(region)
                self.revision += 1
                self._event("placed", "Successful placement cleared its source area", region=region)

    def clear(self):
        with self.lock:
            self.regions.clear()
            self.revision += 1
            self._event("cleared", "Operator cleared repick history")

    def filter_batch(self, batch):
        """Recheck saved/prefetched candidates without changing their identities/order."""
        kept = []
        for candidate in batch.candidates:
            region = self.blocked(candidate.position_m)
            if region is None:
                kept.append(candidate)
            else:
                self._event("excluded", rejection_reason(region), batch_id=batch.identifier,
                            candidate_id=candidate.identifier)
        return replace(batch, candidates=tuple(kept),
                       blacklist_only=(not kept and (batch.blacklist_only
                                                     or bool(batch.candidates))))
