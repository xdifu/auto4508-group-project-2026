#!/usr/bin/env python3
"""Load and validate Part 2 GPS waypoint files."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

from .geo_utils import GeoPoint, haversine_distance_m, wgs84_to_enu

_PASS_SIDE = {"right", "left", "none"}


@dataclass(frozen=True)
class MissionPolicy:
    policy_id: str
    goal_offset_lateral: float
    goal_offset_longitudinal: float
    arrival_radius_override: Optional[float]
    confirm_frames: int


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    lat: float
    lon: float
    alt: float
    x: float
    y: float
    z: float
    arrival_radius: float
    pass_side: str
    yaw_hint: Optional[float]
    policy: str
    photo_required: bool
    notes: str
    is_home_return: bool = False


@dataclass(frozen=True)
class MissionPlan:
    mission_id: str
    datum: GeoPoint
    origin: Optional[GeoPoint]
    return_home: bool
    waypoints: Sequence[Waypoint]
    policies: Dict[str, MissionPolicy]
    bounds: Sequence[GeoPoint]
    home: Optional[GeoPoint]

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)


class WaypointLoader:
    def __init__(
        self,
        waypoints_file: str,
        policy_file: str,
        max_segment_distance_m: float = 200.0,
    ) -> None:
        self._waypoints_path = Path(waypoints_file)
        self._policy_path = Path(policy_file)
        if not self._waypoints_path.exists():
            raise RuntimeError(f"waypoints_file does not exist: {self._waypoints_path}")
        if not self._policy_path.exists():
            raise RuntimeError(f"policy_file does not exist: {self._policy_path}")
        self.max_segment_distance_m = max(1.0, float(max_segment_distance_m))

        self._config = self._load_yaml(self._waypoints_path)
        self._policies = self._load_policies(self._policy_path)
        self.mission_id = str(self._config.get("mission_id", "")).strip()
        if not self.mission_id:
            raise RuntimeError("waypoints_gps.yaml: missing required field `mission_id`.")

        self.return_home = bool(self._config.get("return_home", True))
        self.origin = self._load_optional_origin(self._config)
        self.bounds = self._load_bounds(self._config.get("mission_bounds"))
        self.yaml_datum = self._load_optional_point(self._config.get("datum"), "datum")
        self.yaml_home = self._load_optional_point(self._config.get("home"), "home")
        self._raw_waypoints = self._load_raw_waypoints(self._config.get("waypoints"))

    @staticmethod
    def _load_yaml(path: Path):
        try:
            with path.open("r", encoding="utf-8") as fp:
                return yaml.safe_load(fp) or {}
        except yaml.YAMLError as exc:
            raise RuntimeError(f"YAML parse error in {path}: {exc}") from exc

    @staticmethod
    def _require_float(value, field_name: str) -> float:
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"{field_name}: expected float, got {value!r}") from exc

    def _load_optional_point(self, block, name: str) -> Optional[GeoPoint]:
        if block is None:
            return None
        if not isinstance(block, dict):
            raise RuntimeError(f"{name}: expected map with lat/lon/(alt), got {type(block).__name__}")
        lat = self._require_float(block.get("lat"), f"{name}.lat")
        lon = self._require_float(block.get("lon"), f"{name}.lon")
        alt = self._require_float(block.get("alt", 0.0), f"{name}.alt")
        self._validate_lat_lon(lat, lon, name)
        return GeoPoint(lat=lat, lon=lon, alt=alt)

    def _load_optional_origin(self, config) -> Optional[GeoPoint]:
        if "origin_lat" not in config or "origin_lon" not in config:
            return None
        lat = self._require_float(config.get("origin_lat"), "origin_lat")
        lon = self._require_float(config.get("origin_lon"), "origin_lon")
        alt = self._require_float(config.get("origin_alt", 0.0), "origin_alt")
        self._validate_lat_lon(lat, lon, "origin")
        return GeoPoint(lat=lat, lon=lon, alt=alt)

    def _load_bounds(self, bounds_block) -> List[GeoPoint]:
        if bounds_block is None:
            return []
        if isinstance(bounds_block, dict):
            points_block = bounds_block.get("polygon", [])
        else:
            points_block = bounds_block
        if not isinstance(points_block, list) or len(points_block) < 3:
            raise RuntimeError("mission_bounds: expected at least 3 points.")
        points = []
        for i, raw in enumerate(points_block):
            point = self._load_optional_point(raw, f"mission_bounds[{i}]")
            if point is None:
                raise RuntimeError(f"mission_bounds[{i}]: expected point, got null")
            points.append(point)
        return points

    @staticmethod
    def _validate_lat_lon(lat: float, lon: float, field_name: str) -> None:
        if not (-90.0 <= lat <= 90.0):
            raise RuntimeError(f"{field_name}: latitude out of range [-90, 90], got {lat}")
        if not (-180.0 <= lon <= 180.0):
            raise RuntimeError(f"{field_name}: longitude out of range [-180, 180], got {lon}")

    def _load_policies(self, policy_path: Path) -> Dict[str, MissionPolicy]:
        data = self._load_yaml(policy_path)
        raw_policies = data.get("policies", [])
        if isinstance(raw_policies, dict):
            raw_policies = [dict(policy_id=k, **v) for k, v in raw_policies.items()]
        if not isinstance(raw_policies, list) or not raw_policies:
            raise RuntimeError("mission_policy.yaml: expected non-empty `policies` list.")
        result: Dict[str, MissionPolicy] = {}
        for idx, raw in enumerate(raw_policies):
            if not isinstance(raw, dict):
                raise RuntimeError(f"policies[{idx}]: expected map, got {type(raw).__name__}")
            policy_id = str(raw.get("id", raw.get("policy_id", ""))).strip()
            if not policy_id:
                raise RuntimeError(f"policies[{idx}]: missing `id`.")
            goal_offset = raw.get("goal_offset", {})
            if not isinstance(goal_offset, dict):
                raise RuntimeError(f"policies[{idx}].goal_offset: expected map.")
            lateral = self._require_float(goal_offset.get("lateral", 0.0), f"policies[{idx}].goal_offset.lateral")
            longitudinal = self._require_float(
                goal_offset.get("longitudinal", 0.0),
                f"policies[{idx}].goal_offset.longitudinal",
            )
            arrival_overrides = raw.get("arrival_overrides", {})
            if not isinstance(arrival_overrides, dict):
                raise RuntimeError(f"policies[{idx}].arrival_overrides: expected map.")
            radius_override = arrival_overrides.get("radius")
            if radius_override is not None:
                radius_override = self._require_float(radius_override, f"policies[{idx}].arrival_overrides.radius")
            confirm_frames = int(arrival_overrides.get("confirm_frames", 3))
            result[policy_id] = MissionPolicy(
                policy_id=policy_id,
                goal_offset_lateral=lateral,
                goal_offset_longitudinal=longitudinal,
                arrival_radius_override=radius_override,
                confirm_frames=max(1, confirm_frames),
            )
        if "default" not in result:
            raise RuntimeError("mission_policy.yaml: missing required policy `default`.")
        return result

    def _load_raw_waypoints(self, raw_waypoints) -> List[dict]:
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise RuntimeError("waypoints_gps.yaml: `waypoints` must be a non-empty list.")
        loaded = []
        ids = set()
        for idx, raw in enumerate(raw_waypoints):
            if not isinstance(raw, dict):
                raise RuntimeError(f"waypoints[{idx}]: expected map.")
            waypoint_id = str(raw.get("id", "")).strip()
            if not waypoint_id:
                raise RuntimeError(f"waypoints[{idx}].id is required.")
            if waypoint_id in ids:
                raise RuntimeError(f"waypoints[{idx}].id duplicated: {waypoint_id}")
            ids.add(waypoint_id)

            lat = self._require_float(raw.get("lat"), f"waypoints[{idx}].lat")
            lon = self._require_float(raw.get("lon"), f"waypoints[{idx}].lon")
            alt = self._require_float(raw.get("alt", 0.0), f"waypoints[{idx}].alt")
            self._validate_lat_lon(lat, lon, f"waypoints[{idx}]")

            arrival_radius = self._require_float(raw.get("arrival_radius", 1.5), f"waypoints[{idx}].arrival_radius")
            if arrival_radius < 1.0 or arrival_radius > 2.0:
                raise RuntimeError(
                    f"waypoints[{idx}].arrival_radius out of range [1.0, 2.0], got {arrival_radius}"
                )
            pass_side = str(raw.get("pass_side", "right")).strip().lower()
            if pass_side not in _PASS_SIDE:
                raise RuntimeError(f"waypoints[{idx}].pass_side expected one of {_PASS_SIDE}, got {pass_side}")
            policy = str(raw.get("policy", "default")).strip()
            if policy not in self._policies:
                raise RuntimeError(f"waypoints[{idx}].policy refers to unknown policy `{policy}`")
            loaded.append(
                {
                    "id": waypoint_id,
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "arrival_radius": arrival_radius,
                    "pass_side": pass_side,
                    "yaw_hint": raw.get("yaw_hint"),
                    "policy": policy,
                    "photo_required": bool(raw.get("photo_required", True)),
                    "notes": str(raw.get("notes", "")),
                }
            )
        self._validate_segment_distances(loaded)
        return loaded

    def _validate_segment_distances(self, raw_waypoints: List[dict]) -> None:
        prev = None
        for idx, waypoint in enumerate(raw_waypoints):
            point = GeoPoint(lat=waypoint["lat"], lon=waypoint["lon"], alt=waypoint["alt"])
            if prev is not None:
                d = haversine_distance_m(prev, point)
                if d > self.max_segment_distance_m:
                    raise RuntimeError(
                        f"waypoints[{idx - 1}] -> waypoints[{idx}] distance={d:.2f}m exceeds max_segment_distance="
                        f"{self.max_segment_distance_m:.2f}m"
                    )
            prev = point

    @staticmethod
    def _point_in_polygon(point: GeoPoint, polygon: Sequence[GeoPoint]) -> bool:
        x = point.lon
        y = point.lat
        inside = False
        n = len(polygon)
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]
            x1, y1 = p1.lon, p1.lat
            x2, y2 = p2.lon, p2.lat
            intersects = ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-12) + x1)
            if intersects:
                inside = not inside
        return inside

    def _check_bounds(self, points: Sequence[Tuple[str, GeoPoint]]) -> None:
        if not self.bounds:
            return
        for name, point in points:
            if not self._point_in_polygon(point, self.bounds):
                raise RuntimeError(f"{name} is outside mission_bounds polygon.")

    def build_plan(self, datum: GeoPoint, fallback_home: Optional[GeoPoint] = None) -> MissionPlan:
        self._validate_lat_lon(datum.lat, datum.lon, "datum")
        home = self.yaml_home if self.yaml_home is not None else fallback_home
        if self.return_home and home is None:
            raise RuntimeError("return_home=true but no home provided in YAML and no fallback_home provided.")

        bounds_check_points = [
            (f"waypoint:{w['id']}", GeoPoint(lat=w["lat"], lon=w["lon"], alt=w["alt"]))
            for w in self._raw_waypoints
        ]
        if home is not None:
            bounds_check_points.append(("home", home))
        self._check_bounds(bounds_check_points)

        projected: List[Waypoint] = []
        for raw in self._raw_waypoints:
            wp_geo = GeoPoint(lat=raw["lat"], lon=raw["lon"], alt=raw["alt"])
            ex, ny, up = wgs84_to_enu(wp_geo, datum)
            policy = self._policies[raw["policy"]]
            radius = (
                policy.arrival_radius_override
                if policy.arrival_radius_override is not None
                else raw["arrival_radius"]
            )
            projected.append(
                Waypoint(
                    waypoint_id=raw["id"],
                    lat=wp_geo.lat,
                    lon=wp_geo.lon,
                    alt=wp_geo.alt,
                    x=ex,
                    y=ny,
                    z=up,
                    arrival_radius=radius,
                    pass_side=raw["pass_side"],
                    yaw_hint=float(raw["yaw_hint"]) if raw["yaw_hint"] is not None else None,
                    policy=raw["policy"],
                    photo_required=raw["photo_required"],
                    notes=raw["notes"],
                    is_home_return=False,
                )
            )

        if self.return_home and home is not None:
            hx, hy, hz = wgs84_to_enu(home, datum)
            home_policy = "final_return" if "final_return" in self._policies else "default"
            projected.append(
                Waypoint(
                    waypoint_id="HOME_RETURN",
                    lat=home.lat,
                    lon=home.lon,
                    alt=home.alt,
                    x=hx,
                    y=hy,
                    z=hz,
                    arrival_radius=1.0,
                    pass_side="none",
                    yaw_hint=None,
                    policy=home_policy,
                    photo_required=False,
                    notes="auto-appended return-home waypoint",
                    is_home_return=True,
                )
            )

        return MissionPlan(
            mission_id=self.mission_id,
            datum=datum,
            origin=self.origin,
            return_home=self.return_home,
            waypoints=projected,
            policies=self._policies,
            bounds=self.bounds,
            home=home,
        )
