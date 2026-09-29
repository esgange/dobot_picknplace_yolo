"""Arming gates use advertised canonical services; no inference or robot calls."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.action import GoalResponse

from robot_controller.controller import RobotController
from robot_controller.state_machine import ControllerStateMachine
from robot_controller.tray_client import TrayClient
from test_lifecycle_v2 import reservation
from test_placement import operation_node


@pytest.mark.parametrize('action,providers', [
    ('pick', [('item_teach', '/')]), ('pick', [('item_detect', '/')]),
    ('place', [('tray_teach', '/')]), ('place', [('tray_detect', '/')]),
])
@pytest.mark.parametrize('advertised', [True, False])
def test_readiness_requires_advertised_service_and_one_allowed_provider(
        action, providers, advertised):
    client = SimpleNamespace(service_is_ready=Mock(return_value=advertised), call_async=Mock())
    node = SimpleNamespace(candidates=SimpleNamespace(client=client),
                           _service_providers=Mock(return_value=providers))
    node.check_detector_owner = lambda: RobotController.check_detector_owner(node)
    tray = object.__new__(TrayClient)
    tray.node, tray.client = node, client
    node.trays = tray
    assert RobotController._perception_ready(node, action) is advertised
    client.call_async.assert_not_called()
    if not advertised:
        node._service_providers.assert_not_called()


@pytest.mark.parametrize('action,providers', [
    ('pick', []), ('place', []),
    ('pick', [('item_teach', '/station')]), ('place', [('tray_teach', '/station')]),
    ('pick', [('unknown', '/')]), ('place', [('unknown', '/')]),
    ('pick', [('item_teach', '/'), ('item_detect', '/')]),
    ('place', [('tray_teach', '/'), ('tray_detect', '/')]),
])
def test_unavailable_foreign_or_duplicate_providers_are_not_ready(action, providers):
    client = SimpleNamespace(service_is_ready=lambda: True)
    node = SimpleNamespace(candidates=SimpleNamespace(client=client),
                           _service_providers=lambda _: providers)
    node.check_detector_owner = lambda: RobotController.check_detector_owner(node)
    tray = object.__new__(TrayClient)
    tray.node, tray.client = node, client
    node.trays = tray
    assert not RobotController._perception_ready(node, action)


def test_graph_query_failure_reports_unavailable_without_breaking_status():
    client = SimpleNamespace(service_is_ready=lambda: True)
    node = SimpleNamespace(candidates=SimpleNamespace(client=client),
                           check_detector_owner=Mock(side_effect=RuntimeError('graph changed')))
    assert not RobotController._perception_ready(node, 'pick')


@pytest.mark.parametrize('action', ['pick', 'place'])
def test_goal_checks_current_readiness_before_reserving_any_operation(action):
    if action == 'pick':
        node, calls = reservation('READY')
        configuration_id = 'active'
    else:
        node = operation_node()
        node.machine = ControllerStateMachine(initial='HOLDING')
        node.configuration.configuration_id = 'bound'
        configuration_id = 'bound'
        calls = []
        node._begin_operation = calls.append
    node._perception_ready = Mock(return_value=False)
    assert RobotController._reserve_goal(node, action, configuration_id) == GoalResponse.REJECT
    assert not calls
    node._perception_ready.return_value = True
    assert RobotController._reserve_goal(node, action, configuration_id) == GoalResponse.ACCEPT
    assert calls == [action]
