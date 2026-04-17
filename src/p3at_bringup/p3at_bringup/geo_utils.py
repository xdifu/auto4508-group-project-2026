#!/usr/bin/env python3
"""Geodesy helpers for Part 2 GPS <-> local map projection."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# WGS84 ellipsoid constants.
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_B = _WGS84_A * (1.0 - _WGS84_F)
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)
_WGS84_EP2 = (_WGS84_A * _WGS84_A - _WGS84_B * _WGS84_B) / (_WGS84_B * _WGS84_B)


@dataclass(frozen=True)
class GeoPoint:
    lat: float
    lon: float
    alt: float = 0.0


def _rad(deg: float) -> float:
    return math.radians(deg)


def _deg(rad: float) -> float:
    return math.degrees(rad)


def geodetic_to_ecef(lat: float, lon: float, alt: float = 0.0) -> Tuple[float, float, float]:
    """Convert geodetic coordinates (deg, deg, m) to ECEF (m)."""
    lat_r = _rad(lat)
    lon_r = _rad(lon)
    sin_lat = math.sin(lat_r)
    cos_lat = math.cos(lat_r)
    sin_lon = math.sin(lon_r)
    cos_lon = math.cos(lon_r)

    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt) * cos_lat * cos_lon
    y = (n + alt) * cos_lat * sin_lon
    z = (n * (1.0 - _WGS84_E2) + alt) * sin_lat
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> GeoPoint:
    """Convert ECEF coordinates (m) to geodetic coordinates (deg, deg, m)."""
    p = math.sqrt(x * x + y * y)
    if p < 1e-9:
        lat = math.copysign(math.pi / 2.0, z)
        lon = 0.0
        alt = abs(z) - _WGS84_B
        return GeoPoint(_deg(lat), _deg(lon), alt)

    lon = math.atan2(y, x)
    theta = math.atan2(z * _WGS84_A, p * _WGS84_B)
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    lat = math.atan2(
        z + _WGS84_EP2 * _WGS84_B * sin_t * sin_t * sin_t,
        p - _WGS84_E2 * _WGS84_A * cos_t * cos_t * cos_t,
    )

    sin_lat = math.sin(lat)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    alt = p / math.cos(lat) - n
    return GeoPoint(_deg(lat), _deg(lon), alt)


def wgs84_to_enu(point: GeoPoint, datum: GeoPoint) -> Tuple[float, float, float]:
    """Project WGS84 coordinates to local ENU using datum as the map origin."""
    x, y, z = geodetic_to_ecef(point.lat, point.lon, point.alt)
    x0, y0, z0 = geodetic_to_ecef(datum.lat, datum.lon, datum.alt)
    dx = x - x0
    dy = y - y0
    dz = z - z0

    lat0 = _rad(datum.lat)
    lon0 = _rad(datum.lon)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


def enu_to_wgs84(east: float, north: float, up: float, datum: GeoPoint) -> GeoPoint:
    """Inverse projection: local ENU to WGS84."""
    lat0 = _rad(datum.lat)
    lon0 = _rad(datum.lon)
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    dx = -sin_lon0 * east - sin_lat0 * cos_lon0 * north + cos_lat0 * cos_lon0 * up
    dy = cos_lon0 * east - sin_lat0 * sin_lon0 * north + cos_lat0 * sin_lon0 * up
    dz = cos_lat0 * north + sin_lat0 * up

    x0, y0, z0 = geodetic_to_ecef(datum.lat, datum.lon, datum.alt)
    return ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)


def haversine_distance_m(a: GeoPoint, b: GeoPoint) -> float:
    """Great-circle distance (meters) between two latitude/longitude points."""
    lat1 = _rad(a.lat)
    lon1 = _rad(a.lon)
    lat2 = _rad(b.lat)
    lon2 = _rad(b.lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    sin_dlat = math.sin(dlat / 2.0)
    sin_dlon = math.sin(dlon / 2.0)
    c = 2.0 * math.asin(math.sqrt(sin_dlat * sin_dlat + math.cos(lat1) * math.cos(lat2) * sin_dlon * sin_dlon))
    return _WGS84_A * c


def bearing_rad(a: GeoPoint, b: GeoPoint) -> float:
    """Initial bearing from point a to b in radians, map-compatible ENU yaw."""
    lat1 = _rad(a.lat)
    lon1 = _rad(a.lon)
    lat2 = _rad(b.lat)
    lon2 = _rad(b.lon)
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(y, x)
