"""meta.yaml schema validation.

Implements Section 4 of GrassVIO-Dataset-Spec-DRAFT.md:
required fields, allowed enum values, basic type checks, and a few
cross-field consistency warnings (matrix cell vs. motion flags,
sequence_id vs. directory name).
"""

import os

import yaml

# Fields every meta.yaml must carry (Section 4).
REQUIRED_FIELDS = [
    "sequence_id",
    "date",
    "location",
    "operator",
    "camera_motion",
    "vegetation_motion",
    "matrix_cell",
    "wind_source",
    "wind_mean_ms",
    "wind_max_ms",
    "wind_beaufort",
    "wind_direction_rel_camera",
    "wind_height_m",
    "illumination",
    "mount",
    "camera_height_m",
    "view",
    "approx_scene",
    "duration_s",
    "num_rgb_frames",
    "num_imu_samples",
    "calibration_session",
    "reference_type",
]

# Optional free-text / note fields.
OPTIONAL_FIELDS = [
    "wind_notes",
    "wind_instrument",
    "wind_direction_deg",
    "wind_fan",
    "illumination_notes",
    "reference_notes",
    "observer_notes",
    "imu_gyro_hz",
    "imu_accel_hz",
    "fw_version",
    "sdk_version",
    "global_time_enabled",
    "emitter_enabled",
]

# Allowed values for enum fields (Section 4 comments).
ENUMS = {
    "camera_motion": {"still", "moving"},
    "vegetation_motion": {"still", "moving"},
    "matrix_cell": {"A", "B", "C", "D"},
    "wind_source": {"none", "fan", "natural"},
    "wind_direction_rel_camera": {"headwind", "crosswind", "tailwind"},
    "illumination": {"stable", "changing"},
    "mount": {"tripod", "handheld-rigid"},
    "view": {"oblique", "top-down"},
    "reference_type": {
        "tripod-zero-motion",
        "controlled-rotation",
        "loop-return",
        "repeatable-path",
        "none",
    },
}

# Fields that must parse as numbers (floats ok).
NUMERIC_FIELDS = {
    "camera_height_m",
    "duration_s",
    "wind_mean_ms",
    "wind_max_ms",
    "wind_direction_deg",
    "wind_height_m",
    "imu_gyro_hz",
    "imu_accel_hz",
}

# Fields that must parse as integers.
INT_FIELDS = {"num_rgb_frames", "num_imu_samples", "wind_beaufort"}

# Expected (camera_motion, vegetation_motion) per matrix cell (Section 5).
CELL_CONSISTENCY = {
    "A": ("still", "still"),
    "B": ("still", "moving"),
    "C": ("moving", "still"),
    "D": ("moving", "moving"),
}


def _is_number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def validate_meta(meta_path, seq_dir_name=None):
    """Validate a meta.yaml file.

    Returns a dict with keys:
      status   : "pass" | "fail" | "warn"   ("warn" = schema ok, warnings only)
      errors   : list of fatal problems
      warnings : list of non-fatal inconsistencies
      meta     : parsed mapping (or None if unparseable)
    """
    errors = []
    warnings = []
    meta = None

    if not os.path.isfile(meta_path):
        return {
            "status": "fail",
            "errors": ["meta.yaml not found at %s" % meta_path],
            "warnings": warnings,
            "meta": None,
        }

    try:
        with open(meta_path, "r") as f:
            meta = yaml.safe_load(f)
    except yaml.YAMLError as e:
        return {
            "status": "fail",
            "errors": ["meta.yaml is not valid YAML: %s" % e],
            "warnings": warnings,
            "meta": None,
        }

    if not isinstance(meta, dict):
        return {
            "status": "fail",
            "errors": ["meta.yaml did not parse to a mapping"],
            "warnings": warnings,
            "meta": None,
        }

    # 1. Required fields present and non-empty.
    for field in REQUIRED_FIELDS:
        if field not in meta or meta[field] is None or (
            isinstance(meta[field], str) and not meta[field].strip()
        ):
            errors.append("missing required field: %s" % field)

    # 2. Enum values.
    for field, allowed in ENUMS.items():
        if field in meta and meta[field] is not None:
            val = str(meta[field]).strip()
            if val not in allowed:
                errors.append(
                    "field '%s' has invalid value '%s' (allowed: %s)"
                    % (field, val, sorted(allowed))
                )

    # 3. Numeric / integer types.
    for field in NUMERIC_FIELDS:
        if field in meta and meta[field] is not None and not _is_number(meta[field]):
            errors.append("field '%s' must be numeric, got %r" % (field, meta[field]))
    for field in INT_FIELDS:
        if field in meta and meta[field] is not None and not (
            isinstance(meta[field], int) and not isinstance(meta[field], bool)
        ):
            errors.append("field '%s' must be an integer, got %r" % (field, meta[field]))

    # 4. Cross-field consistency (warnings, not fatal).
    cell = str(meta.get("matrix_cell", "")).strip()
    cam = str(meta.get("camera_motion", "")).strip()
    veg = str(meta.get("vegetation_motion", "")).strip()
    if cell in CELL_CONSISTENCY:
        exp_cam, exp_veg = CELL_CONSISTENCY[cell]
        if cam in ("still", "moving") and veg in ("still", "moving"):
            if (cam, veg) != (exp_cam, exp_veg):
                warnings.append(
                    "matrix_cell '%s' expects camera=%s/vegetation=%s, "
                    "but meta says camera=%s/vegetation=%s"
                    % (cell, exp_cam, exp_veg, cam, veg)
                )

    if seq_dir_name and "sequence_id" in meta and meta["sequence_id"]:
        if str(meta["sequence_id"]).strip() != seq_dir_name:
            warnings.append(
                "sequence_id '%s' does not match directory name '%s'"
                % (meta["sequence_id"], seq_dir_name)
            )

    # 5. Unknown fields are tolerated but noted.
    known = set(REQUIRED_FIELDS) | set(OPTIONAL_FIELDS)
    for field in meta:
        if field not in known:
            warnings.append("unknown field '%s' (kept, not validated)" % field)

    status = "fail" if errors else ("warn" if warnings else "pass")
    return {"status": status, "errors": errors, "warnings": warnings, "meta": meta}
