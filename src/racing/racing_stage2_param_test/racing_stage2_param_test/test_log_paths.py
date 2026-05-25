"""Stage2 参数测试日志路径（dev_ws/log/stage2_param_test/<中文场景名>/）。"""

import os
from pathlib import Path

from racing_stage2_param_test.ring_track import SUMMARY_FOLDER_ZH, scenario_folder_name


def workspace_root() -> Path:
    here = Path(__file__).resolve()
    for parent in [here.parent, *here.parents]:
        if (parent / 'src').is_dir() and (
            (parent / 'AGENTS.md').is_file()
            or (parent / 'install').is_dir()
            or (parent / 'build').is_dir()
        ):
            return parent
    return Path.cwd()


def test_log_root() -> Path:
    root = workspace_root() / 'log' / 'stage2_param_test'
    root.mkdir(parents=True, exist_ok=True)
    return root


def summary_log_dir() -> Path:
    path = test_log_root() / SUMMARY_FOLDER_ZH
    path.mkdir(parents=True, exist_ok=True)
    return path


def ensure_test_log_dir() -> Path:
    return test_log_root()


def scenario_log_dir(scenario: str) -> Path:
    folder = scenario_folder_name(scenario)
    path = test_log_root() / folder
    path.mkdir(parents=True, exist_ok=True)
    return path


def trajectory_csv_path(scenario: str = '') -> str:
    if scenario:
        return str(scenario_log_dir(scenario) / 'trajectory.csv')
    return str(summary_log_dir() / 'trajectory.csv')


def debug_log_path(scenario: str = '') -> str:
    if scenario:
        return str(scenario_log_dir(scenario) / 'debug.log')
    return str(summary_log_dir() / 'debug.log')


def ring_plot_path(scenario: str = '') -> str:
    if scenario:
        return str(scenario_log_dir(scenario) / 'ring_plot.png')
    return str(summary_log_dir() / 'ring_plot.png')


def scenario_result_path(scenario: str = '') -> str:
    if scenario:
        return str(scenario_log_dir(scenario) / 'result.txt')
    return str(summary_log_dir() / 'result.txt')
