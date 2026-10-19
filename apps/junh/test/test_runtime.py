"""Shutdown conversion races are suppressed only after context termination."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from junh_master.ros import runtime


@pytest.mark.parametrize('context_alive',[True,False])
def test_runtime_failure_is_visible_unless_context_has_shutdown(monkeypatch,context_alive):
    cleaned=[]
    node=SimpleNamespace(destroy_node=lambda:cleaned.append('node'))
    monkeypatch.setattr(runtime.rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(runtime.rclpy,'ok',lambda:context_alive)
    monkeypatch.setattr(runtime.rclpy,'try_shutdown',lambda:cleaned.append('context'))
    def spin(node):raise RuntimeError('Subscription conversion failure')
    monkeypatch.setattr(runtime.rclpy,'spin',spin)
    if context_alive:
        with pytest.raises(RuntimeError):runtime.run(lambda:node)
    else:runtime.run(lambda:node)
    assert cleaned==['node','context']
