#!/usr/bin/env python3
"""并发扫描 IPv4 网段，并汇总能够 ping 通的设备。"""

from __future__ import annotations

import argparse
import ipaddress
import math
import platform
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass


DEFAULT_NETWORK = "192.168.31.0/24"
LATENCY_PATTERN = re.compile(r"(?:time|时间)[=<]\s*([\d.]+)\s*ms", re.IGNORECASE)


@dataclass(frozen=True)
class PingResult:
    address: str
    online: bool
    latency_ms: float
    error: str = ""


def parse_network(value: str) -> ipaddress.IPv4Network:
    """接受完整网段，也接受 192.168.31 这种简写。"""
    value = value.strip()
    if re.fullmatch(r"(?:\d{1,3}\.){2}\d{1,3}", value):
        value += ".0/24"
    elif re.fullmatch(r"(?:\d{1,3}\.){3}\d{1,3}", value):
        value += "/32"

    try:
        network = ipaddress.ip_network(value, strict=False)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"无效网段：{value}（{exc}）") from exc

    if not isinstance(network, ipaddress.IPv4Network):
        raise argparse.ArgumentTypeError("目前仅支持 IPv4 网段")
    if network.prefixlen < 16:
        raise argparse.ArgumentTypeError(
            "网段过大，最多允许扫描 /16（65536 个地址）"
        )
    return network


def positive_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是数字") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return number


def positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是整数") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return number


def build_ping_command(address: str, timeout: float) -> list[str]:
    system = platform.system().lower()
    timeout_ms = max(1, math.ceil(timeout * 1000))

    if system == "windows":
        return ["ping", "-n", "1", "-w", str(timeout_ms), address]
    if system == "darwin":
        return ["ping", "-n", "-c", "1", "-W", str(timeout_ms), address]
    if system in {"linux", "freebsd", "openbsd", "netbsd"}:
        # Linux 的 ping -W 单位为秒，其他 BSD 通常为毫秒；这里主要照顾 Linux。
        wait = str(math.ceil(timeout))
        return ["ping", "-n", "-c", "1", "-W", wait, address]
    return ["ping", "-c", "1", address]


def ping_once(address: str, timeout: float) -> PingResult:
    command = build_ping_command(address, timeout)
    started = time.perf_counter()

    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout + 2,
            check=False,
        )
    except FileNotFoundError:
        return PingResult(address, False, 0.0, "找不到 ping 命令")
    except subprocess.TimeoutExpired:
        elapsed_ms = (time.perf_counter() - started) * 1000
        return PingResult(address, False, elapsed_ms, "ping 命令超时")
    except OSError as exc:
        return PingResult(address, False, 0.0, f"无法执行 ping：{exc}")

    elapsed_ms = (time.perf_counter() - started) * 1000
    output = completed.stdout or ""
    if completed.returncode == 0:
        match = LATENCY_PATTERN.search(output)
        latency = float(match.group(1)) if match else elapsed_ms
        return PingResult(address, True, latency)

    lowered = output.lower()
    permission_errors = (
        "operation not permitted",
        "permission denied",
        "requires root",
        "权限不足",
        "拒绝访问",
    )
    if any(text in lowered for text in permission_errors):
        error = "没有发送 ICMP 数据包的权限"
    elif completed.returncode == 2:
        first_line = next(
            (line.strip() for line in output.splitlines() if line.strip()),
            "",
        )
        error = first_line or "ping 命令执行失败（退出码 2）"
    else:
        error = ""
    return PingResult(address, False, elapsed_ms, error)


def scan_network(
    network: ipaddress.IPv4Network, timeout: float, workers: int
) -> list[PingResult]:
    addresses = [str(address) for address in network.hosts()]
    if not addresses:
        return []

    results: list[PingResult] = []
    worker_count = min(workers, len(addresses))
    progress_step = max(1, len(addresses) // 20)

    with ThreadPoolExecutor(max_workers=worker_count) as executor:
        futures = {
            executor.submit(ping_once, address, timeout): address
            for address in addresses
        }
        for index, future in enumerate(as_completed(futures), start=1):
            results.append(future.result())
            if index == len(addresses) or index % progress_step == 0:
                print(
                    f"\r扫描进度：{index}/{len(addresses)}",
                    end="",
                    file=sys.stderr,
                    flush=True,
                )

    print(file=sys.stderr)
    return sorted(
        results,
        key=lambda result: ipaddress.IPv4Address(result.address),
    )


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="并发 ping 一个网段，并汇总在线设备。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例：\n"
            "  python ping_sweep.py\n"
            "  python ping_sweep.py --network 192.168.1.0/24\n"
            "  python ping_sweep.py --network 192.168.31 --timeout 1.5\n"
            "  python ping_sweep.py --network 10.0.0.0/24 --include-offline"
        ),
    )
    parser.add_argument(
        "-n",
        "--network",
        type=parse_network,
        default=parse_network(DEFAULT_NETWORK),
        help=f"要扫描的 IPv4 网段或 CIDR（默认：{DEFAULT_NETWORK}）",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=positive_float,
        default=1.0,
        help="单个设备的 ping 超时秒数（默认：1.0）",
    )
    parser.add_argument(
        "-j",
        "--workers",
        type=positive_int,
        default=64,
        help="并发扫描数（默认：64）",
    )
    parser.add_argument(
        "--include-offline",
        action="store_true",
        help="同时列出离线或不可达的设备",
    )
    return parser


def main() -> int:
    args = create_parser().parse_args()

    if shutil.which("ping") is None:
        print("错误：系统中找不到 ping 命令。", file=sys.stderr)
        return 2

    total_hosts = max(0, args.network.num_addresses - 2)
    if args.network.prefixlen >= 31:
        total_hosts = args.network.num_addresses

    print(f"扫描网段：{args.network}", flush=True)
    print(
        f"目标数量：{total_hosts}  |  并发数：{args.workers}  "
        f"|  单机超时：{args.timeout:g} 秒",
        flush=True,
    )
    print("正在扫描，请稍候...", flush=True)

    started = time.perf_counter()
    try:
        results = scan_network(args.network, args.timeout, args.workers)
    except KeyboardInterrupt:
        print("\n扫描已取消。", file=sys.stderr)
        return 130

    elapsed = time.perf_counter() - started
    online = [result for result in results if result.online]
    offline = [result for result in results if not result.online]

    print()
    if online:
        print(f"在线设备（{len(online)} 台）：")
        for result in online:
            print(f"  {result.address:<15} {result.latency_ms:>8.1f} ms")
    else:
        print("在线设备：未发现可 ping 通的设备。")

    if args.include_offline and offline:
        print()
        print(f"离线或不可达（{len(offline)} 台）：")
        for result in offline:
            suffix = f"  [{result.error}]" if result.error else ""
            print(f"  {result.address}{suffix}")

    failed_calls = [result for result in offline if result.error]
    print()
    print(f"扫描完成，用时 {elapsed:.2f} 秒")
    print(f"汇总：在线 {len(online)} 台，离线或不可达 {len(offline)} 台")
    permission_failed = False
    if failed_calls:
        permission_failed = any("权限" in result.error for result in failed_calls)
        print(
            f"提示：{len(failed_calls)} 个 ping 调用未正常执行，"
            "上述离线数量可能不准确。",
            file=sys.stderr,
        )
    if permission_failed:
        print(
            "请检查当前用户是否有发送 ICMP "
            "数据包的权限（Linux 可能需要 root 或 CAP_NET_RAW）。",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
