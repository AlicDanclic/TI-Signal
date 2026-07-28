#!/usr/bin/env python3
"""
Vivado 项目增量同步脚本（最终版）
- 自动查找 prj/xilinx 下的唯一 .xpr 工程文件
- 支持 property.json 中 prjName.PL 读取（可选），但实际以自动查找结果为准
- 扫描 user/{src,sim,data,ip,bd} 目录，对比上次版本生成增量 Tcl
- 包含稳定机制：exit 命令、stdin=DEVNULL、超时、错误处理
- .xci 文件添加后自动升级 IP 并生成 synthesis/simulation 目标
"""

import json
import os
import sys
import subprocess
import tempfile
from typing import Dict, List, Tuple, Optional

# user 子目录与 Vivado fileset 对应关系
CATEGORY_FILESET_MAP = {
    "src": "sources_1",
    "ip": "sources_1",
    "bd": "sources_1",
    "sim": "sim_1",
    "data": "constrs_1",
}
USER_SUBDIRS = list(CATEGORY_FILESET_MAP.keys())

# Vivado 执行超时（秒），可根据工程规模调整
VIVADO_TIMEOUT = 600


def read_json(filepath: str) -> dict:
    """读取 JSON 文件，文件不存在则返回空字典。"""
    if not os.path.isfile(filepath):
        return {}
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def write_json(filepath: str, data: dict) -> None:
    """写入 JSON 文件，自动创建上级目录。"""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def to_posix(rel_path: str) -> str:
    """统一转换为正斜杠路径。"""
    return rel_path.replace(os.sep, "/")


def scan_user_files(project_root: str) -> Dict[str, List[str]]:
    """
    扫描 user/ 下所有子目录，按分类返回文件列表。
    每个文件路径为相对于项目根目录的正斜杠路径。
    """
    result = {subdir: [] for subdir in USER_SUBDIRS}
    user_dir = os.path.join(project_root, "user")
    if not os.path.isdir(user_dir):
        print("警告：user/ 目录不存在，没有用户文件。")
        return result

    for subdir in USER_SUBDIRS:
        full = os.path.join(user_dir, subdir)
        if not os.path.isdir(full):
            continue
        for root, _, files in os.walk(full):
            for file in files:
                abs_path = os.path.join(root, file)
                rel = os.path.relpath(abs_path, project_root)
                result[subdir].append(to_posix(rel))
    return result


def compute_diff(
    current: Dict[str, List[str]], previous: Dict[str, List[str]]
) -> Tuple[List[Tuple[str, str]], List[Tuple[str, str]]]:
    """
    对比当前文件列表与上次记录，返回 (新增列表, 删除列表)。
    每个元素为 (分类, 相对路径)。
    """
    curr_set = set((cat, f) for cat, files in current.items() for f in files)
    prev_set = set((cat, f) for cat, files in previous.items() for f in files)
    added = sorted(curr_set - prev_set, key=lambda x: x[1])
    removed = sorted(prev_set - curr_set, key=lambda x: x[1])
    return added, removed


def find_vivado(preferred: Optional[str] = None) -> str:
    """
    确定 Vivado 可执行文件路径。
    优先级：传入参数 > 环境变量 VIVADO_PATH > 系统 PATH 中的 vivado
    """
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
            # Windows 下允许 .bat 后缀
            if os.path.isfile(cand + ".bat"):
                return cand + ".bat"
            continue
        found = shutil.which(cand)
        if found:
            return found
    print("错误：未找到 Vivado 可执行文件。请设置环境变量 VIVADO_PATH 或确保 vivado 在 PATH 中。")
    sys.exit(1)


def check_vivado_version(vivado_cmd: str) -> bool:
    """运行 vivado -version 进行快速连通性检查。"""
    try:
        r = subprocess.run(
            [vivado_cmd, "-version"],
            capture_output=True,
            text=True,
            timeout=30
        )
        if r.returncode == 0:
            print("Vivado 版本：")
            print(r.stdout.strip())
            return True
        else:
            print("无法获取 Vivado 版本：", r.stderr)
            return False
    except Exception as e:
        print(f"版本检测异常：{e}")
        return False


def find_xpr(project_root: str) -> str:
    """
    自动在 prj/xilinx 下查找唯一的 .xpr 文件。
    找到多个则报错退出。
    """
    xilinx_dir = os.path.join(project_root, "prj", "xilinx")
    if not os.path.isdir(xilinx_dir):
        print(f"错误：目录 {xilinx_dir} 不存在，无法找到 Vivado 工程。")
        sys.exit(1)

    xpr_files = []
    for root, _, files in os.walk(xilinx_dir):
        for f in files:
            if f.lower().endswith(".xpr"):
                abs_path = os.path.join(root, f)
                rel = os.path.relpath(abs_path, project_root)
                xpr_files.append(to_posix(rel))

    if len(xpr_files) == 0:
        print(f"错误：在 {xilinx_dir} 下未找到任何 .xpr 文件。")
        sys.exit(1)
    if len(xpr_files) > 1:
        print(f"错误：在 prj/xilinx 下找到多个 .xpr 文件：{xpr_files}")
        print("请确保只有一个工程，或手动指定 .xpr 路径。")
        sys.exit(1)

    found = xpr_files[0]
    print(f"自动定位工程文件：{found}")
    return found


def generate_tcl(
    xpr_rel: str,
    added: List[Tuple[str, str]],
    removed: List[Tuple[str, str]],
    project_root: str
) -> str:
    """
    生成 Vivado Tcl 脚本。
    包含打开工程、删除文件、添加文件（含 IP 升级/生成目标、BD 处理）、保存并退出。
    """
    xpr_abs = os.path.abspath(os.path.join(project_root, xpr_rel)).replace(os.sep, "/")

    lines = [
        # 打开工程，并捕获可能错误
        f'if {{[catch {{open_project "{xpr_abs}"}} result]}} {{',
        f'  puts "ERROR: $result"',
        f'  exit 1',
        f'}}',
        ""
    ]

    # 删除文件
    for cat, rel in removed:
        abs_file = os.path.abspath(os.path.join(project_root, rel)).replace(os.sep, "/")
        lines.append(f'remove_files -quiet "{abs_file}"')
    if removed:
        lines.append("")

    # 添加文件
    for cat, rel in added:
        abs_file = os.path.abspath(os.path.join(project_root, rel)).replace(os.sep, "/")
        fileset = CATEGORY_FILESET_MAP.get(cat, "sources_1")
        lines.append(f'add_files -fileset {fileset} -quiet "{abs_file}"')

        # 处理 .xci IP 核：升级并生成综合/仿真目标
        if rel.lower().endswith(".xci"):
            lines.append("update_ip_catalog -quiet")
            lines.append(f'upgrade_ip [get_ips "{abs_file}"] -quiet')
            lines.append(f'generate_target {{synthesis simulation}} [get_ips "{abs_file}"] -quiet')

        # 处理 .bd Block Design：打开并生成 HDL 包装器
        if rel.lower().endswith(".bd"):
            lines.append(f'open_bd_design "{abs_file}" -quiet')
            lines.append(f'make_wrapper -files [get_files "{abs_file}"] -top -quiet')

    lines.append("")
    lines.append("save_project -quiet")
    lines.append('puts "Vivado sync completed."')
    lines.append("exit")  # 关键：避免进入交互模式
    return "\n".join(lines)


def run_vivado(vivado_cmd: str, tcl_content: str, project_root: str) -> bool:
    """
    将 Tcl 写入临时文件并调用 Vivado 执行。
    返回是否成功（无 ERROR 且返回码为 0）。
    """
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".tcl", delete=False, encoding="utf-8"
        ) as f:
            f.write(tcl_content)
            tmp = f.name

        cmd = [
            vivado_cmd,
            "-mode", "tcl",
            "-source", tmp,
            "-notrace",
            "-nolog",
            "-nojournal"
        ]
        print(f"执行命令：{' '.join(cmd)}")
        print(f"超时设置：{VIVADO_TIMEOUT} 秒")

        result = subprocess.run(
            cmd,
            cwd=project_root,
            stdin=subprocess.DEVNULL,   # 防止等待标准输入
            capture_output=True,
            text=True,
            timeout=VIVADO_TIMEOUT
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
        print(f"❌ Vivado 执行超时（{VIVADO_TIMEOUT} 秒），可增大 VIVADO_TIMEOUT 后重试。")
        return False
    except FileNotFoundError:
        print(f"❌ 找不到 Vivado 可执行文件：{vivado_cmd}")
        return False
    except Exception as e:
        print(f"❌ 运行 Vivado 时发生异常：{e}")
        return False
    finally:
        # 清理临时文件
        if tmp and os.path.exists(tmp):
            try:
                os.unlink(tmp)
            except OSError:
                pass


def main():
    project_root = os.getcwd()
    print(f"项目根目录：{project_root}")

    # 1. 自动定位 .xpr 工程文件
    xpr_path = find_xpr(project_root)

    # 2. 扫描当前用户文件
    current_files = scan_user_files(project_root)
    total = sum(len(v) for v in current_files.values())
    print(f"当前扫描到 {total} 个文件")

    # 3. 读取上次同步状态
    version_file = os.path.join(project_root, ".vscode", "version.json")
    version_data = read_json(version_file)
    previous_files = version_data.get("files", {}) if version_data else {}
    previous_vivado_bin = version_data.get("vivadoBinPath") if version_data else None

    # 4. 计算差异
    added, removed = compute_diff(current_files, previous_files)
    print(f"新增：{len(added)} 个，删除：{len(removed)} 个")

    if not added and not removed:
        print("没有文件变更，无需同步。")
        # 如果 version.json 中缺少 Vivado 路径，则补全
        if not version_data or not version_data.get("vivadoBinPath"):
            vivado = find_vivado(previous_vivado_bin)
            write_json(version_file, {"files": current_files, "vivadoBinPath": vivado})
            print("已更新 version.json 中的 Vivado 路径。")
        return

    # 5. 定位 Vivado 并验证
    vivado = find_vivado(previous_vivado_bin)
    print(f"Vivado 路径：{vivado}")
    if not check_vivado_version(vivado):
        print("请确认 Vivado 安装正确，或修改 .vscode/version.json 中的 vivadoBinPath。")
        sys.exit(1)

    # 6. 生成 Tcl 脚本
    tcl = generate_tcl(xpr_path, added, removed, project_root)

    # 7. 执行同步
    success = run_vivado(vivado, tcl, project_root)

    # 8. 更新状态文件
    if success:
        write_json(version_file, {"files": current_files, "vivadoBinPath": vivado})
        print("同步成功，version.json 已更新。")
    else:
        print("同步失败，未修改 version.json。")
        sys.exit(1)


if __name__ == "__main__":
    main()