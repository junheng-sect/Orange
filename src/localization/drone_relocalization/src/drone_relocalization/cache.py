"""Fingerprint and directory checks shared by preflight and the full loader."""
import hashlib
import json
import os
import zipfile
import numpy as np
from .settings import load

CACHE_VERSION='indoor-se3-v2-1'

def cache_key(meta_bytes, pcd, cfg):
    digest = hashlib.sha256()
    digest.update(meta_bytes)
    digest.update(CACHE_VERSION.encode())
    digest.update(json.dumps(cfg, sort_keys=True).encode())
    with open(pcd, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            digest.update(chunk)
    return digest.hexdigest()


def cache_valid(folder, cfg=None):
    """Check the v2 fingerprint and directory. NNField.load validates the payload."""
    folder = os.path.abspath(os.path.expanduser(folder))
    try:
        with open(os.path.join(folder, 'meta.json'), 'rb') as stream:
            meta = stream.read()
        floor = json.loads(meta).get('floor')
        if not isinstance(floor, dict) or floor.get('trusted') is not True:
            return False
        with np.load(os.path.join(folder, 'reloc_cache_v2.npz'), allow_pickle=False) as cache:
            required = {'key', 'lo', 'dims', 'grid', 'n', 'good', 'res'}
            return (required.issubset(cache.files)
                    and str(cache['key']) == cache_key(meta, os.path.join(folder, 'map_world.pcd'), cfg or load()))
    except (OSError, ValueError, KeyError, TypeError, AttributeError, EOFError, zipfile.BadZipFile):
        return False
