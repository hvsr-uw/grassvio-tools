#!/usr/bin/env python3
"""Compatibility entry point; implementation lives in ../../stitch-map/."""
import importlib.util
from pathlib import Path
import sys

_source = Path(__file__).resolve().parents[2] / "stitch-map" / "stitch_map.py"
sys.path.insert(0, str(_source.parent))
_spec = importlib.util.spec_from_file_location("_grassvio_stitch_map", _source)
_impl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_impl)
for _name in dir(_impl):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_impl, _name)

if __name__ == "__main__":
    sys.exit(_impl.main())
