#!/usr/bin/env python3
"""Run all CI-controller tests; never invoke a native simulator or Docker job."""
from pathlib import Path
import unittest

if __name__ == '__main__':
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parent), pattern='*Tests.py')
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() and result.testsRun else 1)
