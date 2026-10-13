"""Draw a world-axis orientation indicator over a rendered RGB frame."""
import math

from PIL import Image, ImageDraw


def draw_axis_gizmo(frame, scene, axes='XYZ'):
    """Project world axes using the rendered camera, including free-camera views."""
    camera = scene.camera[0]
    forward, up = camera.forward, camera.up
    right = [forward[1] * up[2] - forward[2] * up[1],
             forward[2] * up[0] - forward[0] * up[2],
             forward[0] * up[1] - forward[1] * up[0]]
    length = math.sqrt(sum(value * value for value in right))
    right = [value / length for value in right]
    panel = Image.new('RGBA', (144, 144), (0, 0, 0, 0))
    draw = ImageDraw.Draw(panel)
    draw.rounded_rectangle((0, 0, 143, 143), radius=12, fill=(15, 18, 22, 210))
    draw.text((12, 10), 'World axes', fill='white')
    origin = (72, 82)
    colors = {'X': '#ff6464', 'Y': '#6ee787', 'Z': '#69b7ff'}
    for axis in axes:
        index = 'XYZ'.index(axis)
        dx, dy = 42 * right[index], -42 * up[index]
        end = (origin[0] + dx, origin[1] + dy)
        color = colors[axis]
        projected_length = math.hypot(dx, dy)
        if projected_length < 3:
            draw.ellipse((end[0] - 4, end[1] - 4, end[0] + 4, end[1] + 4),
                         outline=color, width=2)
            label = (end[0] + 8, end[1] - 8)
        else:
            draw.line((origin, end), fill=color, width=3)
            ux, uy = dx / projected_length, dy / projected_length
            draw.polygon([end,
                          (end[0] - 9 * ux + 4 * uy, end[1] - 9 * uy - 4 * ux),
                          (end[0] - 9 * ux - 4 * uy, end[1] - 9 * uy + 4 * ux)],
                         fill=color)
            label = (end[0] + 9 * ux - 3, end[1] + 9 * uy - 5)
        draw.text(label, '+' + axis, fill=color)
    draw.ellipse((69, 79, 75, 85), fill='white')
    frame.paste(panel, (12, max(0, frame.height - 156)), panel)
    return frame
