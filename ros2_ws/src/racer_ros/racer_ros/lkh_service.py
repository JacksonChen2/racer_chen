"""Compatibility service for RACER's file-oriented LKH invocation."""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess

import rclpy
from rclpy.node import Node

from racer_interfaces.srv import SolveMTSP, SolveTSP


class LkhService(Node):
    def __init__(self) -> None:
        super().__init__("lkh_service")
        self.declare_parameter("drone_id", 1)
        self.declare_parameter("resource_directory", "")
        self.declare_parameter("lkh_executable", "LKH")
        drone_id = self.get_parameter("drone_id").value
        self.create_service(SolveTSP, f"solve_tsp_{drone_id}", self._solve_tsp)
        self.create_service(SolveMTSP, f"solve_mtsp_{drone_id}", self._solve_mtsp)

    def _execute(self, candidates: list[str]) -> bool:
        executable = shutil.which(self.get_parameter("lkh_executable").value)
        root = Path(self.get_parameter("resource_directory").value)
        if executable is None:
            self.get_logger().error("LKH executable was not found")
            return False
        parameter = next((root / name for name in candidates if (root / name).exists()), None)
        if parameter is None:
            self.get_logger().error(f"no LKH parameter file in {root}")
            return False
        completed = subprocess.run([executable, str(parameter)], check=False)
        return completed.returncode == 0

    def _solve_tsp(self, request, response):
        self._execute(["single.par", "tsp.par"])
        response.empty = request.prob
        return response

    def _solve_mtsp(self, request, response):
        self._execute([f"amtsp{request.prob}.par", "mtsp.par"])
        response.empty = request.prob
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LkhService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
