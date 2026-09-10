# -*- coding: utf-8 -*-
"""GIF -> ESP32 嵌入资源转换器 v3（QR 工程表情动画用）
用法: python qr_gif_convert.py <in.gif> <out.bin> [skip] [size]
v3: 自动裁剪内容外沿空白（union bbox 扩为正方形），角色尽量占满屏幕。
全局 256 色调色板 + 索引 RLE。bin 布局同 v2（'GAS2'）。
"""
import sys, struct
from PIL import Image, ImageChops, ImageSequence

MAGIC = 0x32534147  # 'GAS2'

def rle_encode_idx(idx_bytes):
    out = bytearray()
    i = 0
    n = len(idx_bytes)
    while i < n:
        c = idx_bytes[i]
        run = 1
        while i + run < n and idx_bytes[i + run] == c and run < 255:
            run += 1
        out += struct.pack('<BB', c, run)
        i += run
    return out

def main():
    src, dst = sys.argv[1], sys.argv[2]
    skip = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    tw = int(sys.argv[4]) if len(sys.argv) > 4 else 128   # 目标宽
    th = int(sys.argv[5]) if len(sys.argv) > 5 else 128   # 目标高
    im = Image.open(src)

    # 1) 取合成后的完整帧（选中的那些）
    sel = []
    for idx, frame in enumerate(ImageSequence.Iterator(im)):
        if idx % skip == 0:
            sel.append(frame.convert('RGB'))
        if len(sel) >= 64:
            break

    # 2) 自动裁剪：与角落背景色求差的 union bbox，扩为正方形 + 4% 边距
    bg = sel[0].getpixel((2, 2))
    bg_img = Image.new('RGB', sel[0].size, bg)
    bbox = None
    for f in sel:
        b = ImageChops.difference(f, bg_img).getbbox()
        if b:
            bbox = b if bbox is None else (
                min(bbox[0], b[0]), min(bbox[1], b[1]),
                max(bbox[2], b[2]), max(bbox[3], b[3]))
    x0, y0, x1, y1 = bbox
    cw, ch = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
    fit = len(sys.argv) > 6 and sys.argv[6] == 'fit'
    if fit:
        # 等比缩放整帧适配目标区，居中补背景色（不裁内容）
        bg = sel[0].getpixel((2, 2))
        frames = []
        for f in sel:
            r = min(tw / f.width, th / f.height)
            nw, nh = max(1, int(f.width * r)), max(1, int(f.height * r))
            canvas = Image.new('RGB', (tw, th), bg)
            canvas.paste(f.resize((nw, nh), Image.LANCZOS), ((tw - nw) // 2, (th - nh) // 2))
            frames.append(canvas)
        # 跳过裁剪，直接进入量化
        box = None
    # 按目标宽高比取最大裁剪框（居中于内容，严格限制在帧内，保持比例）
    aspect = tw / th
    W, H = sel[0].size
    bh = min(H, max(int(cw / aspect), ch))
    bw = int(bh * aspect)
    if bw > W:
        bw = W
        bh = min(H, int(bw / aspect))
    bx = min(max(cx - bw // 2, 0), W - bw)
    by = min(max(cy - bh // 2, 0), H - bh)
    box = (bx, by, bx + bw, by + bh)
    print(f'裁剪: 原始{sel[0].size} -> bbox{bbox} -> 区域{box} (目标{tw}x{th})')

    # 3) 裁剪缩放 + 全局调色板量化 + 索引 RLE
    if fit:
        crops = frames
    else:
        crops = [f.crop(box).resize((tw, th), Image.LANCZOS) for f in sel]
    strip = Image.new('RGB', (tw, th * len(crops)))
    for k, c in enumerate(crops):
        strip.paste(c, (0, th * k))
    pal_img = strip.quantize(colors=256, method=Image.MEDIANCUT)

    frames = []
    for c in crops:
        q = c.quantize(palette=pal_img, dither=Image.Dither.NONE)
        idx_bytes = q.tobytes()
        delay = 60   # 统一 60ms 目标间隔（实际受 SPI 推屏耗时限制）
        rle = rle_encode_idx(idx_bytes)
        data, is_rle = (rle, 1) if len(rle) < len(idx_bytes) else (idx_bytes, 0)
        frames.append((data, delay, is_rle))

    pal = pal_img.getpalette()
    pal_rgb565 = []
    for i in range(256):
        r = pal[i * 3] >> 3
        g = pal[i * 3 + 1] >> 2
        b = pal[i * 3 + 2] >> 3
        pal_rgb565.append((r << 11) | (g << 5) | b)

    count = len(frames)
    table_off = 12 + 512
    hdr = struct.pack('<IHHHH', MAGIC, count, tw, th, 256)
    data_off = table_off + count * 12
    with open(dst, 'wb') as f:
        f.write(hdr)
        f.write(struct.pack('<256H', *pal_rgb565))
        off = data_off
        for d, delay, is_rle in frames:
            f.write(struct.pack('<IIHH', off, len(d), delay, is_rle))
            off += len(d)
        for d, _, _ in frames:
            f.write(d)

    total = data_off + sum(len(d) for d, _, _ in frames)
    print(f'帧数={count} 尺寸={tw}x{th} bin总大小={total / 1024:.0f}KB')

if __name__ == '__main__':
    main()
