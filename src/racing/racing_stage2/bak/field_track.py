"""field_track.py �?场测赛道平面加载，取�?direct_inertial_tester 的硬编码 build_ring_plan()�?
用法�?    plan = load_plan('config/field_track_clockwise.yaml', 'clockwise')
    # �?[{'type':'turn','angle_deg':95.0,'description':'rect_enter_align'},
    #    {'type':'move','distance_m':1.10,'speed':ring_v,'allow_detour':True,
    #     'description':'rect_first_leg'}, ...]
"""

import os
import yaml


def load_plan(yaml_path: str, direction: str,
              ring_linear_speed: float = 0.20,
              allow_detour: bool = True) -> list:
    """�?field_track_*.yaml 加载赛道计划段序列�?
    参数�?        yaml_path:  YAML 文件路径
        direction:  行驶方向（仅用于校验�?        ring_linear_speed:  直行段默认速度
        allow_detour:  是否允许直行段插入避�?
    返回�?        �?build_ring_plan() 格式相同的段列表（list[dict]），
        如果 YAML 包含 coordinate_system=world，则 move 段会包含 start/end 绝对坐标�?    """
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(
            f'field_track yaml not found: {yaml_path}')

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    file_dir = str(data.get('direction', ''))
    if file_dir and file_dir != direction:
        raise ValueError(
            f'field_track direction mismatch: file={file_dir}, requested={direction}')

    raw_segments = data.get('segments', [])
    if not raw_segments:
        raise ValueError(f'no segments in {yaml_path}')

    # 检查是否使用世界坐标系
    coordinate_system = str(data.get('coordinate_system', 'relative')).lower()
    is_world = (coordinate_system == 'world')

    # 提取起点（世界坐标系时使用）
    start_pose = data.get('start_pose', {})

    plan = []
    for seg in raw_segments:
        entry = dict(seg)  # shallow copy
        seg_type = entry.get('type', '')
        
        if seg_type == 'move':
            entry['speed'] = entry.get('speed', ring_linear_speed)
            if allow_detour:
                entry['allow_detour'] = True
            
            # 如果是世界坐标系，保�?target 绝对坐标
            if is_world and 'target' in entry:
                entry['coordinate_system'] = 'world'
        
        elif seg_type == 'turn':
            # 如果是世界坐标系，标�?            if is_world:
                entry['coordinate_system'] = 'world'
        
        plan.append(entry)

    # 如果是世界坐标系，返回起点信息（用于初始化）
    if is_world and start_pose:
        # �?plan 前插入起点信息（作为元数据）
        return plan, start_pose
    
    return plan


def resolve_yaml_path(package_dir: str, direction: str, custom_path: str = '', use_world: bool = True) -> str:
    """解析 field_track YAML 路径�?
    优先级：
        1. custom_path（如果非空）
        2. 如果 use_world=True，优先选择 field_track_{direction}_world.yaml
        3. 否则根据 direction 自动选择 config/field_track_{direction}.yaml
    """
    if custom_path:
        return custom_path
    direction = str(direction).lower()
    
    # 优先使用世界坐标版本
    if use_world:
        if direction.startswith('counter'):
            fn = 'field_track_counterclockwise_world.yaml'
        else:
            fn = 'field_track_clockwise_world.yaml'
        world_path = os.path.join(package_dir, 'config', fn)
        if os.path.isfile(world_path):
            return world_path
    
    # 回退到相对坐标版�?    if direction.startswith('counter'):
        fn = 'field_track_counterclockwise.yaml'
    else:
        fn = 'field_track_clockwise.yaml'
    return os.path.join(package_dir, 'config', fn)
