#!/usr/bin/env python
"""不依赖 pytest 的测试入口。

用法（在 perception/ 目录下）：
    python tests/run_all.py
    python tests/run_all.py -v          # 显示每个用例
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    files = sorted(p for p in HERE.glob("test_*.py"))
    passed = failed = 0
    failures = []
    t_start = time.perf_counter()
    for path in files:
        module = load_module(path)
        cases = sorted(name for name in dir(module) if name.startswith("test_") and callable(getattr(module, name)))
        if args.verbose:
            print("\n%s (%d)" % (path.name, len(cases)))
        for name in cases:
            try:
                getattr(module, name)()
                passed += 1
                if args.verbose:
                    print("  PASS  %s" % name)
            except Exception:
                failed += 1
                failures.append((path.name, name, traceback.format_exc()))
                print("  FAIL  %s::%s" % (path.name, name))

    for fname, name, tb in failures:
        print("\n" + "=" * 70)
        print("FAILED %s::%s" % (fname, name))
        print(tb)

    dt = time.perf_counter() - t_start
    print("\n%d passed, %d failed  (%.2fs)" % (passed, failed, dt))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
