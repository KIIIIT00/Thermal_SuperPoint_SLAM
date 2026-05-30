#!/usr/bin/env python3
import argparse
import gzip
import re
import shutil
from pathlib import Path

DESC_RE = re.compile(r'(descriptor:\")([^\"]*)(\")')


def read_text(path: Path) -> str:
    if path.suffix == '.gz':
        with gzip.open(path, 'rt', encoding='utf-8', errors='ignore') as f:
            return f.read()
    return path.read_text(encoding='utf-8', errors='ignore')


def write_text(path: Path, text: str) -> None:
    if path.suffix == '.gz':
        with gzip.open(path, 'wt', encoding='utf-8') as f:
            f.write(text)
    else:
        path.write_text(text, encoding='utf-8')


def infer_type(tokens):
    # OpenCV type id: CV_8U=0, CV_32F=5
    # If any token looks float, treat as CV_32F.
    for t in tokens:
        if any(ch in t for ch in ('.', 'e', 'E')):
            return 5
    return 0


def convert_descriptor(raw: str):
    s = raw.strip()
    if not s:
        return raw, False, None, None
    if s.startswith('dbw3 '):
        return raw, False, None, None

    tokens = s.split()
    cv_type = infer_type(tokens)
    cols = len(tokens)
    converted = f'dbw3 {cv_type} {cols} ' + s
    return converted, True, cv_type, cols


def convert_text(text: str):
    converted_count = 0
    types = set()
    dims = set()

    def _repl(m):
        nonlocal converted_count
        pre, body, post = m.group(1), m.group(2), m.group(3)
        new_body, changed, cv_type, cols = convert_descriptor(body)
        if changed:
            converted_count += 1
            types.add(cv_type)
            dims.add(cols)
        return pre + new_body + post

    out = DESC_RE.sub(_repl, text)
    return out, converted_count, types, dims


def backup_file(path: Path) -> Path:
    bak = path.with_suffix(path.suffix + '.bak')
    if bak.exists():
        idx = 2
        while True:
            cand = path.with_name(path.name + f'.bak{idx}')
            if not cand.exists():
                bak = cand
                break
            idx += 1
    shutil.copy2(path, bak)
    return bak


def main():
    ap = argparse.ArgumentParser(description='Convert vocabulary YAML descriptors to DBoW3 descriptor string format.')
    ap.add_argument('input', type=Path, help='Input vocabulary file (.yml/.yaml/.gz)')
    ap.add_argument('-o', '--output', type=Path, default=None, help='Output path (default: overwrite input)')
    ap.add_argument('--in-place', action='store_true', help='Overwrite input file (creates backup)')
    args = ap.parse_args()

    inp = args.input
    if not inp.exists():
        raise SystemExit(f'Input not found: {inp}')

    text = read_text(inp)
    out_text, converted_count, types, dims = convert_text(text)

    if args.in_place:
        bak = backup_file(inp)
        write_text(inp, out_text)
        out_path = inp
        print(f'backup: {bak}')
    else:
        out_path = args.output if args.output else inp.with_name(inp.name + '.dbow3')
        write_text(out_path, out_text)

    print(f'output: {out_path}')
    print(f'converted_descriptors: {converted_count}')
    if types:
        print(f'cv_types: {sorted(types)}')
    if dims:
        print(f'dims: {sorted(dims)}')


if __name__ == '__main__':
    main()
