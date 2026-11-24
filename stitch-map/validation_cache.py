"""Identify completed measurements by inputs, source code and numeric libraries."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import scipy


def fingerprint(bag, inputs, modules):
    bag = Path(bag)
    digest = hashlib.sha256()
    digest.update(json.dumps([str(bag.resolve()), bag.stat().st_size, bag.stat().st_mtime_ns,
                              cv2.__version__, np.__version__, scipy.__version__]).encode())
    for name in sorted(set(modules) | {'validation_cache'}):
        path = Path(__file__).with_name(name+'.py')
        digest.update(name.encode())
        digest.update(path.read_bytes())
    for path in inputs:
        path = Path(path)
        digest.update(str(path.resolve()).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def completed(path, expected_fingerprint, required=()):
    path = Path(path)
    if not path.exists() or not all(Path(p).exists() for p in required):
        return None
    result = json.loads(path.read_text())
    return result if result.get('source_fingerprint') == expected_fingerprint else None
