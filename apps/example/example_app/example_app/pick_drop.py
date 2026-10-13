"""Fixed, physically verified waypoints for the example lab layout.

The demo picks block_3 at (0.40, -0.33) m, lifts it, and releases it over
that location. Positions, block dimensions, and the arm heading are known.
"""
from dataclasses import dataclass
import math


BLOCK_NAME = 'block_3'
# Six arm joint angles in radians: hover, then descend to the grasp pose.
PICK_PATH = (
    (2.590655897, 1.769112535, -0.865306583, 1.540173443, 0.985481409, 0.854229843),
    (2.590863270, 1.807161547, -0.843562670, 1.580002973, 0.984704848, 0.782479428),
    (2.591072610, 1.846586650, -0.823378440, 1.619654689, 0.985102748, 0.711001635),
    (2.591275953, 1.890081708, -0.807633392, 1.659009914, 0.989059973, 0.640047061),
    (2.591472135, 1.935971563, -0.795091477, 1.697166435, 0.995264234, 0.570593304),
    (2.591660125, 1.984046387, -0.785762312, 1.733881635, 1.003506737, 0.503010345),
    (2.591838070, 2.034053617, -0.779696351, 1.768812784, 1.013563088, 0.437774437),
    (2.592004511, 2.085714503, -0.776925904, 1.801675391, 1.025154677, 0.375314182),
    (2.592153458, 2.139331491, -0.779078358, 1.831127749, 1.038499092, 0.317554698),
)


@dataclass
class Stage:
    name: str
    target: list
    duration: float

    def __post_init__(self):
        self.target = [float(value) for value in self.target]


def plan(scene, initial):
    """Return the fixed sequence; refuse a layout these poses weren't made for."""
    def close(actual, expected, tolerance):
        return len(actual) == len(expected) and all(abs(a-b) <= tolerance for a, b in zip(actual, expected))

    block = next((block for block in scene['blocks'] if block['name'] == BLOCK_NAME), None)
    heading = math.radians(145)/2
    base_quaternion = [math.cos(heading), 0, 0, math.sin(heading)]
    if (block is None or not close(block['position'], [.4, -.33, .0075], .005)
            or not close(scene['block_size'], [.075, .025, .015], 1e-6)
            or not close(scene['robot']['base_position'], [.1, -.08, 0], 1e-6)
            or not (close(scene['robot']['base_quaternion'], base_quaternion, 1e-6)
                    or close(scene['robot']['base_quaternion'], [-v for v in base_quaternion], 1e-6))):
        raise ValueError('The fixed demo expects block_3 at (0.40, -0.33), 7.5 × 2.5 × 1.5 cm blocks, and the original arm base pose. Restore that layout and restart.')
    stages = [Stage('open gripper', [*initial[:6], .035], 1.0),
              Stage('approach block', [*PICK_PATH[0], .035], 4.0)]
    stages += [Stage('descend', [*pose, .035], .4) for pose in PICK_PATH[1:]]
    stages += [Stage('grasp', [*PICK_PATH[-1], 0], 1.5)]
    stages += [Stage('lift', [*pose, 0], .4) for pose in reversed(PICK_PATH[:-1])]
    stages += [Stage('hold above table', [*PICK_PATH[0], 0], 1.0),
               Stage('release block', [*PICK_PATH[0], .035], 1.0),
               Stage('wait for block to land', [*PICK_PATH[0], .035], 1.0),
               Stage('return home', [*initial[:6], .035], 4.0)]
    return BLOCK_NAME, stages
