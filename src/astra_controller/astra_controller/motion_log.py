"""Bounded per-process motion traces, with wall timestamps shared across stages."""
import json
import logging
from logging.handlers import RotatingFileHandler
import os
import time


def motion_trace(stage, identity):
    identity = identity.strip('/').replace('/', '_') or 'root'
    logger = logging.getLogger(f'motion_trace.{stage}.{identity}')
    if not logger.handlers:
        handler = RotatingFileHandler(
            f'/tmp/astra_motion_{stage}_{identity}_{os.getpid()}.jsonl',
            maxBytes=10 * 1024 * 1024, backupCount=2,
        )
        handler.setFormatter(logging.Formatter('%(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False

    def record(**fields):
        logger.info(json.dumps(dict(timestamp=time.time(), stage=stage, **fields), separators=(',', ':')))
    return record
