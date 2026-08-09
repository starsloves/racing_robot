import math
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parents[1]))

from racing_tools.start_corner_pose_diagnostic import compose_map_xy


def test_map_xy_uses_one_rigid_transform():
    x, y = compose_map_xy(0.53, 0.25, math.radians(24.5), 1.906717, -0.999508)
    assert math.isclose(x, 2.679528, abs_tol=1e-6)
    assert math.isclose(y, 0.131189, abs_tol=1e-6)


if __name__ == '__main__':
    test_map_xy_uses_one_rigid_transform()
    print('coordinate formula ok')
