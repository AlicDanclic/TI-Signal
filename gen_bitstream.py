#!/usr/bin/env python3
"""
Vivado 生成 Bitstream 脚本
自动查找 prj/xilinx 下的 .xpr 工程，调用 Vivado 完成综合、实现、生成 .bit 文件。
"""

import argparse
import json
import os
import sys
import subprocess
import tempfile
from typing import Optional

# 默认超时（秒），可在命令行覆盖
DEFAULT_TIMEOUT = 3600  # 1 小时


def read_json(filepath: str) -> dict:
    if not os.path.isfile(filepath):
        return {}
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def find_xpr(project_root: str) -> str:
    """自动在 prj/xilinx 下查找唯一的 .xpr 文件"""
    xilinx_dir = os.path.join(project_root, "prj", "xilinx")
    if not os.path.isdir(xilinx_dir):
        sys.exit(f"错误：目录 {xilinx_dir} 不存在。")

    xpr_files = []
    for root, _, files in os.walk(xilinx_dir):
        for f in files:
            if f.lower().endswith(".xpr"):
                abs_path = os.path.join(root, f)
                rel = os.path.relpath(abs_path, project_root).replace(os.sep, "/")
                xpr_files.append(rel)

    if len(xpr_files) == 0:
        sys.exit(f"错误：在 {xilinx_dir} 下未找到任何 .xpr 文件。")
    if len(xpr_files) > 1:
        print(f"错误：找到多个 .xpr 文件：{xpr_files}")
        sys.exit(1)

    print(f"自动定位工程文件：{xpr_files[0]}")
    return xpr_files[0]


def find_vivado(preferred: Optional[str] = None) -> str:
    """确定 Vivado 可执行文件路径"""
    candidates = []
    if preferred:
        candidates.append(preferred)
    env_val = os.environ.get("VIVADO_PATH")
    if env_val:
        candidates.append(env_val)
    candidates.append("vivado")

    import shutil
    for cand in candidates:
        if os.path.isabs(cand):
            if os.path.isfile(cand):
                return cand
            if os.path.isfile(cand + ".bat"):
                return cand + ".bat"
            continue
        found = shutil.which(cand)
        if found:
            return found
    sys.exit("错误：未找到 Vivado 可执行文件。请设置环境变量 VIVADO_PATH 或确保 vivado 在 PATH 中。")


def generate_tcl(xpr_abs: str,
                 output_bit: str,
                 jobs: int = 8,
                 reset_all: bool = True) -> str:
    """生成 Vivado Tcl 脚本"""
    lines = [
        f'if {{[catch {{open_project "{xpr_abs}"}} result]}} {{',
        f'  puts "ERROR: $result"',
        f'  exit 1',
        f'}}',
        ""
    ]

    if reset_all:
        lines.extend([
            'reset_run synth_1',
            'reset_run impl_1',
            ""
        ])

    lines.extend([
        f'launch_runs synth_1 -jobs {jobs}',
        'wait_on_run synth_1',
        "",
        f'launch_runs impl_1 -to_step write_bitstream -jobs {jobs}',
        'wait_on_run impl_1',
        "",
        'open_run impl_1',
        f'write_bitstream -force "{output_bit}"',
        "",
        'puts "Bitstream generation completed."',
        'exit'
    ])
    return "\n".join(lines)


def run_vivado(vivado_cmd: str, tcl_content: str, project_root: str, timeout: int) -> bool:
    """执行 Tcl 脚本，返回是否成功"""
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".tcl", delete=False, encoding="utf-8"
        ) as f:
            f.write(tcl_content)
            tmp = f.name

        cmd = [vivado_cmd, "-mode", "tcl", "-source", tmp,
               "-notrace", "-nolog", "-nojournal"]
        print(f"执行命令：{' '.join(cmd)}")

        result = subprocess.run(
            cmd,
            cwd=project_root,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=timeout
        )

        combined = (result.stdout or "") + (result.stderr or "")
        if "ERROR:" in combined or result.returncode != 0:
            print("❌ Vivado 返回错误：")
            print(combined)
            return False

        print("✅ Vivado 执行成功")
        if combined.strip():
            print(combined)
        return True

    except subprocess.TimeoutExpired:
        print(f"❌ Vivado 执行超时（{timeout} 秒），请增大 --timeout 值。")
        return False
    except Exception as e:
        print(f"❌ 运行异常：{e}")
        return False
    finally:
        if tmp and os.path.exists(tmp):
            os.unlink(tmp)


def main():
    parser = argparse.ArgumentParser(description="使用 Vivado 项目生成 bitstream 文件")
    parser.add_argument("--project-dir", "-p", default=os.getcwd(),
                        help="项目根目录（默认当前目录）")
    parser.add_argument("--output", "-o", default=None,
                        help="输出 .bit 文件路径（默认 <项目根>/bitstream/<工程名>.bit）")
    parser.add_argument("--jobs", "-j", type=int, default=8,
                        help="并行任务数（默认 8）")
    parser.add_argument("--no-reset", action="store_true",
                        help="不重置 synth_1/impl_1，使用已有结果（增量）")
    parser.add_argument("--vivado", default=None,
                        help="Vivado 可执行文件路径（覆盖 version.json）")
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT,
                        help=f"Vivado 执行超时秒数（默认 {DEFAULT_TIMEOUT}）")
    args = parser.parse_args()

    project_root = os.path.abspath(args.project_dir)
    print(f"项目根目录：{project_root}")

    # 1. 定位 .xpr 文件
    xpr_rel = find_xpr(project_root)
    xpr_abs = os.path.join(project_root, xpr_rel).replace(os.sep, "/")
    project_name = os.path.splitext(os.path.basename(xpr_rel))[0]
    print(f"工程名称：{project_name}")

    # 2. 确定输出路径
    if args.output:
        output_bit = os.path.abspath(args.output)
    else:
        out_dir = os.path.join(project_root, "bitstream")
        os.makedirs(out_dir, exist_ok=True)
        output_bit = os.path.join(out_dir, f"{project_name}.bit")
    output_bit = output_bit.replace(os.sep, "/")
    print(f"输出 bitstream 文件：{output_bit}")

    # 3. 定位 Vivado
    version_file = os.path.join(project_root, ".vscode", "version.json")
    version_data = read_json(version_file)
    previous_vivado = version_data.get("vivadoBinPath") if version_data else None
    vivado_preferred = args.vivado or previous_vivado
    vivado = find_vivado(vivado_preferred)
    print(f"Vivado 路径：{vivado}")

    # 4. 生成 Tcl 脚本
    reset = not args.no_reset
    tcl = generate_tcl(xpr_abs, output_bit, jobs=args.jobs, reset_all=reset)

    # 5. 执行
    success = run_vivado(vivado, tcl, project_root, args.timeout)

    if success:
        print(f"Bitstream 生成成功：{output_bit}")
    else:
        print("Bitstream 生成失败。")
        sys.exit(1)


if __name__ == "__main__":
    main()