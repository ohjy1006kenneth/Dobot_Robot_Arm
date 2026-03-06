"""
Convert ASCII or binary_compressed PCD files to binary PCD.

Usage:
    # Convert specific files:
    python convert_to_binary.py file1.pcd file2.pcd

    # Convert all PCD files in a directory tree:
    python convert_to_binary.py --dir .

    # Convert and save to a different output directory:
    python convert_to_binary.py --dir . --outdir binary_pcds/

Files already in binary format are skipped automatically.
Original files are NOT modified — output is written alongside with
the same name (overwrite) or to --outdir if specified.
"""

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

_PCD_TYPE_MAP = {
    ('F', 4): np.float32, ('F', 8): np.float64,
    ('I', 1): np.int8,    ('I', 2): np.int16,   ('I', 4): np.int32,
    ('U', 1): np.uint8,   ('U', 2): np.uint16,  ('U', 4): np.uint32,
}


def _parse_pcd_header(f):
    header = {}
    while True:
        line_bytes = f.readline()
        line = line_bytes.decode('utf-8', errors='replace').strip()
        if not line or line.startswith('#'):
            continue
        key, *vals = line.split()
        key = key.upper()
        if   key == 'FIELDS': header['fields'] = [v.lower() for v in vals]
        elif key == 'SIZE':   header['size']   = [int(v) for v in vals]
        elif key == 'TYPE':   header['type']   = [v.upper() for v in vals]
        elif key == 'COUNT':  header['count']  = [int(v) for v in vals]
        elif key == 'WIDTH':  header['width']  = int(vals[0])
        elif key == 'HEIGHT': header['height'] = int(vals[0])
        elif key == 'POINTS': header['points'] = int(vals[0])
        elif key == 'DATA':
            header['data'] = vals[0].lower()
            break
    return header


def convert(src: Path, dst: Path):
    with open(src, 'rb') as f:
        hdr = _parse_pcd_header(f)
        raw = f.read()

    fmt = hdr['data']
    if fmt == 'binary':
        print(f"  SKIP  {src}  (already binary)")
        return

    fields = hdr['fields']
    sizes  = hdr['size']
    types  = hdr['type']
    counts = hdr.get('count', [1] * len(fields))
    n_pts  = hdr['points']

    dt_list = []
    for name, tp, sz, cnt in zip(fields, types, sizes, counts):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        dt_list.append((name, np_type) if cnt == 1 else (name, np_type, (cnt,)))
    dtype = np.dtype(dt_list)

    if fmt == 'ascii':
        rows = []
        for line in raw.decode('utf-8', errors='replace').splitlines():
            line = line.strip()
            if line:
                rows.append(tuple(
                    float(v) if t == 'F' else int(float(v))
                    for v, t in zip(line.split(), types)
                ))
        arr = np.array(rows, dtype=dtype)
    elif fmt == 'binary_compressed':
        try:
            import lzf
            comp_size   = struct.unpack_from('<I', raw, 0)[0]
            decomp_size = struct.unpack_from('<I', raw, 4)[0]
            data_bytes  = lzf.decompress(raw[8:8 + comp_size], decomp_size)
            arr = np.frombuffer(data_bytes, dtype=dtype).copy()
        except ImportError:
            print(f"  SKIP  {src}  (binary_compressed requires: pip install lzf)")
            return
    else:
        print(f"  SKIP  {src}  (unknown format: {fmt!r})")
        return

    arr = arr[:n_pts]

    header_lines = [
        'VERSION 0.7',
        f'FIELDS {" ".join(fields)}',
        f'SIZE {" ".join(map(str, sizes))}',
        f'TYPE {" ".join(types)}',
        f'COUNT {" ".join(map(str, counts))}',
        f'WIDTH {n_pts}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {n_pts}',
        'DATA binary',
    ]

    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, 'wb') as f:
        f.write(('\n'.join(header_lines) + '\n').encode('utf-8'))
        f.write(arr.tobytes())

    src_mb = src.stat().st_size / 1e6
    dst_mb = dst.stat().st_size / 1e6
    print(f"  OK    {src}  ({src_mb:.1f} MB)  ->  {dst}  ({dst_mb:.1f} MB)  [{fmt} -> binary]")


def main():
    parser = argparse.ArgumentParser(description="Convert PCD files to binary format.")
    parser.add_argument("files", nargs="*", help="PCD files to convert")
    parser.add_argument("--dir",    help="Convert all .pcd files under this directory")
    parser.add_argument("--outdir", help="Write outputs here instead of overwriting")
    args = parser.parse_args()

    paths = [Path(f) for f in args.files]
    if args.dir:
        paths += sorted(Path(args.dir).rglob("*.pcd"))

    if not paths:
        print("No files specified. Use file args or --dir.")
        sys.exit(1)

    print(f"Converting {len(paths)} file(s)...\n")
    ok = skip = err = 0
    for src in paths:
        if not src.exists():
            print(f"  ERR   {src}  (not found)")
            err += 1
            continue
        if args.outdir:
            dst = Path(args.outdir) / src.name
        else:
            dst = src  # overwrite in place
        try:
            convert(src, dst)
            ok += 1
        except Exception as e:
            print(f"  ERR   {src}  ({e})")
            err += 1

    print(f"\nDone: {ok} converted, {skip} skipped, {err} errors.")


if __name__ == "__main__":
    main()