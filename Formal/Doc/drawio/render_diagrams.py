#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
李萨如图形显示控制装置（2026 电赛 F 题）系统图生成脚本。

同一份场景数据（节点 + 边）同时产出：
  1. .drawio 文件（draw.io 可打开编辑，mxfile/diagram/mxGraphModel/root/mxCell）
  2. .png 渲染图（PIL 2 倍超采样直接绘制，与 drawio 同源同布局）

重复运行幂等覆盖输出。仅依赖 Pillow。
运行：.venv_readme/bin/python Doc/drawio/render_diagrams.py
"""

import math
from pathlib import Path
from xml.sax.saxutils import escape
from xml.dom import minidom

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
DRAWIO_DIR = ROOT / "Doc" / "drawio"
BITMAP_DIR = ROOT / "Bitmap"
FONT_PATH = "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"

S = 2  # PNG 超采样倍数

# ---------------------------------------------------------------- 配色（科研感）
INK = "#1A2733"          # 深色文字
SUBINK = "#5A6B7A"       # 次要文字
WIRE = "#44566B"         # 普通连线
TRACE = "#2F8F5B"        # 波形曲线
GRID = "#DDE3E9"         # 网格
AXIS = "#BEC8D2"         # 坐标轴
WARN = "#B23A48"         # 警示/提示

DOMAIN = {
    "analog":  ("#FDF3E7", "#C2703D"),   # 模拟域
    "digital": ("#EAF1F8", "#1F4E79"),   # 数字域
    "visual":  ("#EAF6EE", "#3E7C59"),   # 视觉/控制域
    "extern":  ("#F2F4F6", "#6B7684"),   # 外部设备
    "warn":    ("#FBEDEF", WARN),        # 警示/提示
}

# ---------------------------------------------------------------- 场景数据结构
# 节点: dict(id, kind, x, y, w, h, lines, fill, stroke, radius, dashed, align, sub)
#   kind: box(圆角矩形) / rect / diamond / ellipse / text / group(虚线分组框) / scope(示波器小图)
#   lines: [(文本, 字号, 是否加粗[, 颜色]), ...]
#   scope 的 sub: diag / circle / inf / bars
# 边: dict(pts=[(x,y)...], color, width, dashed, arrow: end/both/none)


def N(id, kind, x, y, w, h, lines=None, fill=None, stroke=None,
      radius=10, dashed=False, align="c", sub=None):
    return dict(id=id, kind=kind, x=x, y=y, w=w, h=h, lines=lines or [],
                fill=fill, stroke=stroke, radius=radius, dashed=dashed,
                align=align, sub=sub)


def E(pts, color=WIRE, width=1.5, dashed=False, arrow="end"):
    return dict(pts=pts, color=color, width=width, dashed=dashed, arrow=arrow)


def T(x, y, w, h, lines, align="c"):
    """无边框文字节点（边标签、备注等）。"""
    return N("t", "text", x, y, w, h, lines=lines, align=align)


# ---------------------------------------------------------------- 字体
_fonts = {}


def get_font(px):
    if px not in _fonts:
        _fonts[px] = ImageFont.truetype(FONT_PATH, px)
    return _fonts[px]


# ---------------------------------------------------------------- PIL 渲染
class Painter:
    def __init__(self, scene):
        self.scene = scene
        self.W, self.H = scene["w"] * S, scene["h"] * S
        self.img = Image.new("RGB", (self.W, self.H), "white")
        self.d = ImageDraw.Draw(self.img)
        self.warnings = []

    # ---- 基础件 ----
    def dashed_polyline(self, pts, color, width, dash=7, gap=5):
        dash, gap = dash * S, gap * S
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 1e-6:
                continue
            ux, uy = (x2 - x1) / length, (y2 - y1) / length
            t = 0.0
            while t < length:
                t2 = min(t + dash, length)
                self.d.line([(x1 + ux * t, y1 + uy * t),
                             (x1 + ux * t2, y1 + uy * t2)],
                            fill=color, width=width)
                t += dash + gap

    def arrowhead(self, p1, p2, color):
        ang = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        L, A = 10 * S, 0.42
        a1 = (p2[0] - L * math.cos(ang - A), p2[1] - L * math.sin(ang - A))
        a2 = (p2[0] - L * math.cos(ang + A), p2[1] - L * math.sin(ang + A))
        self.d.polygon([p2, a1, a2], fill=color)

    # ---- 文字 ----
    def text_block(self, node):
        lines = node["lines"]
        if not lines:
            return
        slot = lambda sz: sz * 1.4
        total = sum(slot(ln[1]) for ln in lines) - lines[-1][1] * 0.4
        y = (node["y"] + node["h"] / 2) * S - total * S / 2
        for ln in lines:
            t, sz, bold = ln[0], ln[1], ln[2]
            color = ln[3] if len(ln) > 3 else INK
            f = get_font(int(sz * S))
            sw = max(1, int(sz * S / 15)) if bold else 0
            tw = self.d.textlength(t, font=f) + 2 * sw
            if tw > (node["w"] - 10) * S:
                self.warnings.append(
                    f"[{self.scene['name']}] 文字超宽: '{t}' "
                    f"{tw / S:.0f}px > {node['w'] - 10}px (节点 {node['id']})")
            cy = y + slot(sz) * S / 2
            if node["align"] == "l":
                self.d.text(((node["x"] + 8) * S, cy), t, font=f, fill=color,
                            anchor="lm", stroke_width=sw, stroke_fill=color)
            else:
                self.d.text(((node["x"] + node["w"] / 2) * S, cy), t,
                            font=f, fill=color, anchor="mm",
                            stroke_width=sw, stroke_fill=color)
            y += slot(sz) * S

    # ---- 示波器小图（真实参数曲线 + 浅网格）----
    def draw_scope(self, node):
        x, y = node["x"] * S, node["y"] * S
        w, h = node["w"] * S, node["h"] * S
        self.d.rectangle([x, y, x + w, y + h], fill="white",
                         outline=node["stroke"], width=max(1, S))
        for i in range(1, 8):  # 8x8 div 网格
            gx, gy = x + w * i / 8, y + h * i / 8
            c = AXIS if i == 4 else GRID
            self.d.line([(gx, y), (gx, y + h)], fill=c, width=max(1, S // 2))
            self.d.line([(x, gy), (x + w, gy)], fill=c, width=max(1, S // 2))
        lw = 2 * S
        sub = node["sub"]
        if sub == "diag":  # 8x8div 对角线
            self.d.line([(x + 3 * S, y + h - 3 * S), (x + w - 3 * S, y + 3 * S)],
                        fill=TRACE, width=lw)
        elif sub == "circle":  # 直径 8div 圆
            m = 4 * S
            self.d.ellipse([x + m, y + m, x + w - m, y + h - m],
                           outline=TRACE, width=lw)
        elif sub == "inf":  # 水平 ∞: x=A sin t, y=A sin 2t（左右双环，与题目“水平∞”一致）
            a, b = (w / 2 - 5 * S), (h / 2 - 5 * S)
            cx, cy = x + w / 2, y + h / 2
            pts = [(cx + a * math.sin(t), cy - b * math.sin(2 * t))
                   for t in [2 * math.pi * i / 720 for i in range(721)]]
            self.d.line(pts, fill=TRACE, width=lw)
        elif sub == "bars":  # Y 峰峰 2/4/6/8 div 四条递增竖线
            for i, k in enumerate([2, 4, 6, 8]):
                bh = (h - 14 * S) * k / 8
                bx = x + w * (1.7 + 1.55 * i) / 8
                cy = y + h / 2
                self.d.rectangle([bx - 2.5 * S, cy - bh / 2,
                                  bx + 2.5 * S, cy + bh / 2], fill=TRACE)

    # ---- 节点 ----
    def draw_node_shape(self, n):
        x, y = n["x"] * S, n["y"] * S
        w, h = n["w"] * S, n["h"] * S
        kind = n["kind"]
        if kind == "text":
            return
        if kind == "scope":
            self.draw_scope(n)
            return
        if kind == "group":
            self.dashed_polyline([(x, y), (x + w, y), (x + w, y + h),
                                  (x, y + h), (x, y)], n["stroke"], max(1, S))
            return
        if kind == "diamond":
            pts = [(x + w / 2, y), (x + w, y + h / 2),
                   (x + w / 2, y + h), (x, y + h / 2)]
            self.d.polygon(pts, fill=n["fill"])
            self.d.line(pts + [pts[0]], fill=n["stroke"], width=max(2, S),
                        joint="curve")
            return
        if kind == "ellipse":
            self.d.ellipse([x, y, x + w, y + h], fill=n["fill"],
                           outline=n["stroke"], width=max(2, S))
            return
        # box / rect
        if kind == "box":
            self.d.rounded_rectangle([x, y, x + w, y + h],
                                     radius=n["radius"] * S, fill=n["fill"],
                                     outline=n["stroke"], width=max(2, S))
        else:
            self.d.rectangle([x, y, x + w, y + h], fill=n["fill"],
                             outline=n["stroke"], width=max(2, S))

    # ---- 边 ----
    def draw_edge(self, e):
        pts = [(px * S, py * S) for px, py in e["pts"]]
        width = max(2, int(e["width"] * S))
        if e["dashed"]:
            self.dashed_polyline(pts, e["color"], width)
        else:
            self.d.line(pts, fill=e["color"], width=width, joint="curve")
        if e["arrow"] in ("end", "both"):
            self.arrowhead(pts[-2], pts[-1], e["color"])
        if e["arrow"] == "both":
            self.arrowhead(pts[1], pts[0], e["color"])

    def render(self):
        sc = self.scene
        # 标题
        tf = get_font(sc["title_size"] * S)
        sw = max(1, sc["title_size"] * S // 15)
        self.d.text((self.W / 2, (sc["title_y"] + sc["title_size"] * 0.75) * S),
                    sc["title"], font=tf, fill=INK, anchor="mm",
                    stroke_width=sw, stroke_fill=INK)
        # 分组虚线框 → 节点形状 → 边 → 文字
        for n in sc["nodes"]:
            if n["kind"] == "group":
                self.draw_node_shape(n)
        for n in sc["nodes"]:
            if n["kind"] != "group":
                self.draw_node_shape(n)
        for e in sc["edges"]:
            self.draw_edge(e)
        for n in sc["nodes"]:
            self.text_block(n)
        return self.img


# ---------------------------------------------------------------- drawio XML 生成
def xml_escape_attr(s):
    return escape(s, {'"': "&quot;"})


def value_html(lines):
    """多行文本 → drawio value：换行用 &#10;，加粗用 <b>，整体 XML 转义。"""
    parts = []
    for ln in lines:
        t = ln[0]
        if ln[2]:
            t = f"<b>{t}</b>"
        parts.append(t)
    return xml_escape_attr("\n".join(parts))


def node_style(n):
    fill = n["fill"] or "none"
    stroke = n["stroke"] or "none"
    fs = int(n["lines"][0][1]) if n["lines"] else 12
    common = (f"whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};"
              f"fontColor={INK};fontSize={fs};align=center;"
              f"verticalAlign=middle;")
    if n["kind"] == "box":
        return f"rounded=1;arcSize=8;{common}"
    if n["kind"] == "diamond":
        return f"rhombus;{common}"
    if n["kind"] == "ellipse":
        return f"ellipse;{common}"
    return f"rounded=0;{common}"


def cell_vertex(cid, value, style, x, y, w, h):
    return (f'<mxCell id="{cid}" value="{value}" style="{style}" '
            f'vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" '
            f'as="geometry"/></mxCell>')


def scope_cells(n):
    """scope 节点的 drawio 近似基元：对角线=line 边、圆=ellipse、∞=两个并排椭圆、
    幅度=四个递增高度矩形。"""
    x, y, w, h, sub = n["x"], n["y"], n["w"], n["h"], n["sub"]
    cid = n["id"]
    cells = [cell_vertex(cid, "", n["style"], x, y, w, h)]
    tstyle = f"fillColor={TRACE};strokeColor={TRACE};html=1;"
    if sub == "diag":
        cells.append(
            f'<mxCell id="{cid}_l" style="endArrow=none;html=1;'
            f'strokeColor={TRACE};strokeWidth=2;" edge="1" parent="1">'
            f'<mxGeometry relative="1" as="geometry">'
            f'<mxPoint x="{x + 4}" y="{y + h - 4}" as="sourcePoint"/>'
            f'<mxPoint x="{x + w - 4}" y="{y + 4}" as="targetPoint"/>'
            f'</mxGeometry></mxCell>')
    elif sub == "circle":
        st = f"ellipse;fillColor=none;strokeColor={TRACE};strokeWidth=2;html=1;"
        cells.append(cell_vertex(f"{cid}_c", "", st, x + 5, y + 5, w - 10, h - 10))
    elif sub == "inf":
        st = f"ellipse;fillColor=none;strokeColor={TRACE};strokeWidth=2;html=1;"
        cells.append(cell_vertex(f"{cid}_c0", "", st,
                                 x + 4, y + 8, w / 2 - 6, h - 16))
        cells.append(cell_vertex(f"{cid}_c1", "", st,
                                 x + w / 2 + 2, y + 8, w / 2 - 6, h - 16))
    elif sub == "bars":
        for i, k in enumerate([2, 4, 6, 8]):
            bh = (h - 14) * k / 8
            bx = x + w * (1.7 + 1.55 * i) / 8
            cells.append(cell_vertex(f"{cid}_b{i}", "", tstyle,
                                     round(bx - 2.5, 1), round(y + h / 2 - bh / 2, 1),
                                     5, round(bh, 1)))
    return cells


def cell_edge(cid, e):
    pts = e["pts"]
    style = (f"edgeStyle=none;html=1;rounded=0;strokeColor={e['color']};"
             f"strokeWidth={e['width']};")
    style += "endArrow=block;endFill=1;" if e["arrow"] in ("end", "both") \
        else "endArrow=none;"
    style += "startArrow=block;startFill=1;" if e["arrow"] == "both" \
        else "startArrow=none;"
    if e["dashed"]:
        style += "dashed=1;dashPattern=8 6;"
    geom = (f'<mxPoint x="{pts[0][0]}" y="{pts[0][1]}" as="sourcePoint"/>'
            f'<mxPoint x="{pts[-1][0]}" y="{pts[-1][1]}" as="targetPoint"/>')
    if len(pts) > 2:
        wp = "".join(f'<mxPoint x="{px}" y="{py}"/>'
                     for px, py in pts[1:-1])
        geom += f'<Array as="points">{wp}</Array>'
    return (f'<mxCell id="{cid}" style="{style}" edge="1" parent="1">'
            f'<mxGeometry relative="1" as="geometry">{geom}</mxGeometry>'
            f'</mxCell>')


def render_drawio(scene, path):
    cells = []
    # 标题（也写进 drawio，保持同布局）
    title_lines = [(scene["title"], scene["title_size"], True)]
    cells.append(cell_vertex(
        "title", value_html(title_lines),
        f"text;html=1;align=center;verticalAlign=middle;fontColor={INK};"
        f"fontSize={scene['title_size']};",
        0, scene["title_y"], scene["w"], scene["title_size"] * 2))
    for idx, n in enumerate(scene["nodes"]):
        n["id"] = f"v{idx}"  # mxCell id 必须唯一（部分节点共用占位 id）
        n["style"] = node_style(n)  # scope 复用
        if n["kind"] == "group":
            cells.append(cell_vertex(
                n["id"], "",
                f"rounded=0;dashed=1;dashPattern=8 6;fillColor=none;"
                f"strokeColor=#9AA4AF;html=1;",
                n["x"], n["y"], n["w"], n["h"]))
        elif n["kind"] == "scope":
            cells.extend(scope_cells(n))
        elif n["kind"] == "text":
            color = n["lines"][0][3] if n["lines"] and len(n["lines"][0]) > 3 \
                else SUBINK
            fs = int(n["lines"][0][1]) if n["lines"] else 11
            align = "left" if n["align"] == "l" else "center"
            cells.append(cell_vertex(
                n["id"], value_html(n["lines"]),
                f"text;html=1;whiteSpace=wrap;align={align};"
                f"verticalAlign=middle;fontColor={color};fontSize={fs};",
                n["x"], n["y"], n["w"], n["h"]))
        else:
            cells.append(cell_vertex(n["id"], value_html(n["lines"]),
                                     n["style"], n["x"], n["y"], n["w"], n["h"]))
    for i, e in enumerate(scene["edges"]):
        cells.append(cell_edge(f"e{i}", e))
    xml = (
        f'<mxfile host="app.diagrams.net" type="device">'
        f'<diagram name="{xml_escape_attr(scene["name"])}">'
        f'<mxGraphModel dx="1000" dy="600" grid="1" gridSize="10" guides="1" '
        f'tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" '
        f'pageWidth="{scene["w"]}" pageHeight="{scene["h"]}" math="0" shadow="0">'
        f'<root><mxCell id="0"/><mxCell id="1" parent="0"/>'
        + "".join(cells) +
        f'</root></mxGraphModel></diagram></mxfile>')
    path.write_text(xml, encoding="utf-8")
    minidom.parse(str(path))  # 合法性校验


# ---------------------------------------------------------------- 图1：系统总体架构
def scene_architecture():
    A, D, V, X = DOMAIN["analog"], DOMAIN["digital"], DOMAIN["visual"], DOMAIN["extern"]
    nodes = [
        N("grp", "group", 270, 140, 895, 420, stroke="#9AA4AF"),
        T(290, 148, 400, 24, [("李萨如图形显示控制装置", 14, True, SUBINK)], align="l"),
        # 外部设备
        N("sig", "box", 50, 270, 150, 110, fill=X[0], stroke=X[1], lines=[
            ("信号源", 15, True), ("正弦输出 1kHz~100kHz", 11, False),
            ("步进 100Hz", 11, False)]),
        N("osc", "box", 1200, 200, 160, 140, fill=X[0], stroke=X[1], lines=[
            ("示波器", 15, True), ("X-Y 模式", 11, False),
            ("X、Y 灵敏度 0.5V/div", 11, False)]),
        N("cam", "box", 1200, 430, 160, 110, fill=X[0], stroke=X[1], lines=[
            ("摄像头", 15, True), ("720p@30fps", 11, False),
            ("V4L2", 11, False)]),
        # 装置内部：主信号链
        N("afe", "box", 310, 270, 135, 110, fill=A[0], stroke=A[1], lines=[
            ("模拟前端", 14, True), ("衰减 / 阻抗匹配 / 驱动", 11, False)]),
        N("adc", "box", 475, 270, 135, 110, fill=A[0], stroke=A[1], lines=[
            ("ADC 3PA1030", 13, True), ("双通道 10bit", 11, False),
            ("50MSPS", 11, False)]),
        N("fpga", "box", 640, 240, 230, 170, fill=D[0], stroke=D[1], lines=[
            ("Zynq-7020 PL（FPGA）", 14, True),
            ("等精度测频 · 波形捕获(181点)", 11, False),
            ("DDS 正弦合成 · 数字移相", 11, False),
            ("二倍频 · 程控幅度", 11, False)]),
        N("ps", "text", 540, 428, 420, 22, lines=[
            ("PS：SD卡启动，FSBL 加载 PL 位流（BOOT.bin）", 10.5, False, SUBINK)]),
        N("dac", "box", 895, 270, 120, 110, fill=A[0], stroke=A[1], lines=[
            ("DAC 3PD5651E", 12, True), ("双通道 10bit", 11, False)]),
        N("obuf", "box", 1040, 270, 110, 110, fill=A[0], stroke=A[1], lines=[
            ("输出缓冲", 13, True), ("幅度调理", 11, False)]),
        # 下方：人机交互 + 视觉闭环
        N("stm32", "box", 380, 660, 270, 130, fill=D[0], stroke=D[1], lines=[
            ("STM32F429 人机交互", 13, True),
            ("（Apollo 开发板）", 11, False),
            ("4.3寸触摸屏波形监视", 11, False),
            ("频率/峰峰值/有效值", 11, False),
            ("三键启动 · LED · 蜂鸣器声光提示", 10.5, False)]),
        N("rpi", "box", 750, 660, 240, 130, fill=V[0], stroke=V[1], lines=[
            ("树莓派 · 视觉闭环", 13, True),
            ("OpenCV 轨迹提取", 11, False),
            ("椭圆拟合 · Chamfer 匹配", 11, False),
            ("非阻塞状态机", 11, False)]),
        # 边标签
        T(620, 68, 160, 20, [("X 轴输入", 11, False, SUBINK)]),
        T(205, 298, 100, 20, [("装置输入", 11, False, SUBINK)]),
        T(1120, 298, 110, 20, [("Y 轴输出", 11, False, SUBINK)]),
        T(1200, 376, 90, 20, [("屏幕图像", 11, False, SUBINK)]),
        T(1060, 700, 90, 20, [("图像帧", 11, False, SUBINK)]),
        T(620, 646, 160, 34, [("UART 115200", 10.5, False, SUBINK),
                              ("16B帧+CRC16", 10.5, False, SUBINK)]),
        T(525, 486, 170, 20, [("UART 命令/波形遥测", 10.5, False, SUBINK)]),
    ]
    edges = [
        E([(125, 270), (125, 95), (1280, 95), (1280, 200)], color="#6B7684"),  # X 轴绕线
        E([(200, 325), (310, 325)], color=WIRE, width=1.8),                     # 装置输入
        E([(445, 325), (475, 325)], color=WIRE, width=1.8),
        E([(610, 325), (640, 325)], color=WIRE, width=1.8),
        E([(870, 325), (895, 325)], color=WIRE, width=1.8),
        E([(1015, 325), (1040, 325)], color=WIRE, width=1.8),
        E([(1150, 325), (1200, 325)], color=WIRE, width=1.8),                   # Y 轴输出
        E([(1280, 340), (1280, 430)], color="#3E7C59"),                         # 屏幕图像
        E([(1280, 540), (1280, 725), (990, 725)], color="#3E7C59"),             # 图像帧
        E([(750, 725), (650, 725)], color=WIRE, arrow="both"),                  # 树莓派↔STM32
        E([(515, 660), (515, 470), (700, 470), (700, 410)],
          color=WIRE, arrow="both"),                                            # STM32↔FPGA
    ]
    return dict(name="system_architecture", w=1400, h=880,
                title="李萨如图形显示控制装置 · 系统总体架构",
                title_size=23, title_y=24, nodes=nodes, edges=edges)


# ---------------------------------------------------------------- 图2：四种工作模式
def scene_modes():
    D, A, V, X = DOMAIN["digital"], DOMAIN["analog"], DOMAIN["visual"], DOMAIN["extern"]
    rows_top = [95, 255, 415, 575]
    centers = [t + 60 for t in rows_top]
    procs = [
        [("直通模式", 14, True), ("x(t) 直接输出", 11, False)],
        [("正交相移", 14, True), ("DDS 数字移相 +90°", 11, False)],
        [("二倍频", 14, True), ("DDS 输出频率 ×2", 11, False)],
        [("程控幅度", 14, True), ("幅度码 k 四档", 11, False)],
    ]
    formulas = [
        [("y(t) = x(t)", 14, False)],
        [("y(t) = A·sin(ωt+90°)", 13, False), ("幅度相等", 11, False)],
        [("y(t) = A·sin(2ωt)", 13, False), ("幅度相等", 11, False)],
        [("y(t) = k·A·sin(ωt+φ)", 13, False), ("k 四档", 11, False)],
    ]
    reqs = [
        [("要求1 · 对角线（5分）", 13, True), ("8×8 div 矩形对角线", 11, False)],
        [("要求2 · 圆（15分）", 13, True), ("直径 8div · 幅差 ≤0.2div", 11, False)],
        [("要求3 · ∞形（25分）", 13, True), ("上下左右对称", 11, False)],
        [("要求4 · 幅度设置（15分）", 13, True), ("Y 峰峰 2/4/6/8 div", 11, False)],
    ]
    subs = ["diag", "circle", "inf", "bars"]
    nodes = [
        N("inp", "box", 40, 300, 180, 130, fill=X[0], stroke=X[1], lines=[
            ("输入 x(t) = A·sin(ωt)", 13, True),
            ("1kHz~100kHz", 11, False), ("步进 100Hz", 11, False)]),
    ]
    edges = []
    for i, (top, ry) in enumerate(zip(rows_top, centers)):
        nodes += [
            N(f"p{i}", "box", 300, top, 180, 120, fill=D[0], stroke=D[1],
              lines=procs[i]),
            N(f"f{i}", "box", 520, top, 200, 120, fill=D[0], stroke=D[1],
              lines=formulas[i]),
            N(f"s{i}", "scope", 760, top - 15, 150, 150, stroke="#6B7684",
              sub=subs[i]),
            N(f"r{i}", "box", 950, top, 300, 120, fill=A[0], stroke=A[1],
              lines=reqs[i]),
        ]
        edges += [
            E([(220, 365), (255, 365), (255, ry), (300, ry)]),
            E([(480, ry), (520, ry)]),
            E([(720, ry), (760, ry)]),
            E([(910, ry), (950, ry)]),
        ]
    nodes.append(N("note5", "box", 40, 745, 1210, 65, fill=V[0], stroke=V[1],
                   radius=8, lines=[
        ("要求5 · 视觉闭环自动模式（40分）：摄像头拍摄示波器屏幕，"
         "一键自动完成对角线 / 圆 / ∞", 12, True),
        ("控制时间 ≤10s · 稳定时间 ≥5s · 完成后声光提示 → 见自动模式控制流程图",
         12, False)]))
    return dict(name="signal_modes", w=1300, h=820,
                title="四种工作模式与李萨如图形原理",
                title_size=23, title_y=22, nodes=nodes, edges=edges)


# ---------------------------------------------------------------- 图3：自动模式状态机
def scene_flow():
    D, V, X, W = DOMAIN["digital"], DOMAIN["visual"], DOMAIN["extern"], DOMAIN["warn"]
    GREEN, GRAY = "#3E7C59", "#6B7684"
    nodes = [
        # 主流程（自上而下）
        N("start", "box", 415, 56, 320, 74, fill=D[0], stroke=D[1], lines=[
            ("按键启动", 15, True),
            ("LEFT=对角线 · DOWN=圆 · RIGHT=∞", 12, False),
            ("STM32 发 EVENT_START", 11, False, SUBINK)]),
        N("coarse", "box", 415, 154, 320, 80, fill=V[0], stroke=V[1], lines=[
            ("COARSE 粗测频", 14, True),
            ("FPGA 输出锯齿探针 0.1/0.5/2ms 三档", 12, False),
            ("拐点周期中位数统计", 12, False)]),
        N("fine", "box", 415, 258, 320, 80, fill=V[0], stroke=V[1], lines=[
            ("FINE_PHASE 精测相", 14, True),
            ("3ms/7ms 双细条探针", 12, False),
            ("圆周平均相位块联合消歧", 12, False)]),
        N("target", "box", 415, 362, 320, 80, fill=V[0], stroke=V[1], lines=[
            ("发送 TARGET", 14, True),
            ("目标号 · 幅度码 · 相位码", 12, False),
            ("32位 DDS 调谐字 f·2³²/50MHz", 12, False)]),
        N("d1", "diamond", 485, 462, 180, 92, fill=DOMAIN["analog"][0],
          stroke=DOMAIN["analog"][1], lines=[("目标 = 圆？", 13, True)]),
        # 分支
        N("sweep", "box", 130, 612, 260, 80, fill=V[0], stroke=V[1], lines=[
            ("CIRCLE_SWEEP 分层扫频", 13, True),
            ("100Hz 网格对齐", 11, False),
            ("低频 tier→高频通道→亚 Hz 微调", 11, False)]),
        N("clock", "box", 130, 716, 260, 96, fill=V[0], stroke=V[1], lines=[
            ("CIRCLE_LOCK 三环伺服", 13, True),
            ("相位 A/B 试探 + 幅度试探", 11, False),
            ("+ 频率积分补偿晶振误差", 11, False)]),
        N("track", "box", 760, 610, 260, 96, fill=V[0], stroke=V[1], lines=[
            ("TRACK 跟踪", 13, True),
            ("相位双向试探（Chamfer 匹配）", 11, False),
            ("幅度按 span_y 缩放至 8div", 11, False)]),
        N("d2", "diamond", 475, 844, 200, 92, fill=DOMAIN["analog"][0],
          stroke=DOMAIN["analog"][1], lines=[("锁定质量达标？", 12, True)]),
        N("locked", "box", 415, 960, 320, 68, fill=V[0], stroke=V[1], lines=[
            ("LOCKED 锁定", 14, True),
            ("STATUS_LOCKED 上报 · 每 0.25s 复查维护", 11, False)]),
        N("alarm", "box", 770, 960, 270, 68, fill=D[0], stroke=D[1], lines=[
            ("声光提示", 13, True),
            ("STM32 驱动 LED + 蜂鸣器（PF8）", 11, False)]),
        # 左侧视觉反馈
        N("cam", "box", 20, 250, 200, 150, fill=V[0], stroke=V[1], lines=[
            ("摄像头 30fps 图像反馈", 12, True),
            ("HSV 轨迹提取", 11, False),
            ("椭圆拟合 · Chamfer 匹配", 11, False),
            ("实时视觉反馈", 11, False, SUBINK)]),
        # 右侧备注
        N("note", "box", 780, 300, 320, 150, fill=W[0], stroke=W[1], lines=[
            ("异常处理与指标", 13, True, WARN),
            ("超时/失败 → 恢复探针输出", 11, False),
            ("STATUS_ERROR 上报", 11, False),
            ("指标：控制时间 ≤10s", 11, False),
            ("稳定时间 ≥5s", 11, False)]),
        # 分支标签
        T(340, 486, 60, 20, [("是", 12, True, GREEN)]),
        T(750, 486, 60, 20, [("否", 12, True, WARN)]),
        T(8, 690, 60, 20, [("否", 12, True, WARN)]),
        T(1018, 896, 60, 20, [("否", 12, True, WARN)]),
        T(585, 940, 60, 20, [("是", 12, True, GREEN)]),
    ]
    edges = [
        E([(575, 130), (575, 154)], color=WIRE, width=1.8),
        E([(575, 234), (575, 258)], color=WIRE, width=1.8),
        E([(575, 338), (575, 362)], color=WIRE, width=1.8),
        E([(575, 442), (575, 462)], color=WIRE, width=1.8),
        # 目标=圆？分支
        E([(485, 508), (260, 508), (260, 612)], color=GREEN, width=1.8),   # 是
        E([(665, 508), (890, 508), (890, 610)], color=WIRE, width=1.8),    # 否
        E([(260, 692), (260, 716)], color=WIRE, width=1.8),
        # 汇合到锁定质量判断
        E([(260, 812), (260, 890), (475, 890)], color=WIRE, width=1.8),
        E([(890, 706), (890, 890), (675, 890)], color=WIRE, width=1.8),
        # 否 → 回边
        E([(510, 922), (45, 922), (45, 750), (130, 750)], color=WARN,
          width=1.6),
        E([(640, 922), (1080, 922), (1080, 655), (1020, 655)], color=WARN,
          width=1.6),
        # 是 → 锁定 → 声光
        E([(575, 936), (575, 960)], color=GREEN, width=1.8),
        E([(735, 994), (770, 994)], color=WIRE, width=1.8),
        # 摄像头实时反馈（虚线）
        E([(90, 400), (90, 764), (130, 764)], color=GREEN, dashed=True),
        E([(170, 400), (170, 605), (740, 605), (740, 640), (760, 640)],
          color=GREEN, dashed=True),
        E([(90, 764), (90, 994), (415, 994)], color=GREEN, dashed=True),
    ]
    return dict(name="auto_control_flow", w=1150, h=1050,
                title="自动模式 · 视觉闭环状态机（要求5）",
                title_size=23, title_y=18, nodes=nodes, edges=edges)


# ---------------------------------------------------------------- 主函数
def main():
    DRAWIO_DIR.mkdir(parents=True, exist_ok=True)
    BITMAP_DIR.mkdir(parents=True, exist_ok=True)
    for build in (scene_architecture, scene_modes, scene_flow):
        scene = build()
        name = scene["name"]
        p = Painter(scene)
        img = p.render()
        png_path = BITMAP_DIR / f"{name}.png"
        img.save(png_path)
        drawio_path = DRAWIO_DIR / f"{name}.drawio"
        render_drawio(scene, drawio_path)
        print(f"[ok] {name}: PNG {img.size[0]}x{img.size[1]} -> {png_path}")
        print(f"[ok] {name}: drawio XML 合法 -> {drawio_path}")
        for w in p.warnings:
            print("  !! " + w)


if __name__ == "__main__":
    main()
