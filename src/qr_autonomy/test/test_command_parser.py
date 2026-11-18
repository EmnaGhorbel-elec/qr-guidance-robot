import pytest

from qr_autonomy.command_parser import command_id, parse_command


def test_single_actions():
    assert parse_command('C07|TURN_L|60').actions == [('TURN_L', 60.0)]
    assert parse_command('C08|turn_r|90').actions == [('TURN_R', 90.0)]
    assert parse_command('C09|MOVE|1.0').actions == [('MOVE', 1.0)]
    assert parse_command('C09|MOVE|1,5').actions == [('MOVE', 1.5)]
    assert parse_command('C11|FOLLOW').actions == [('FOLLOW', 0.0)]
    assert parse_command('C12|STOP').actions == [('STOP', 0.0)]


def test_chain_and_id():
    cmd = parse_command(' C13|TURN_L|60|MOVE|1.0|FOLLOW ')
    assert cmd.cid == 'C13'
    assert cmd.actions == [('TURN_L', 60.0), ('MOVE', 1.0), ('FOLLOW', 0.0)]


@pytest.mark.parametrize('text', [
    '', 'hello', 'C07', 'C07|JUMP|3', 'C07|TURN_L', 'C07|TURN_L|abc', 'C07|TURN_L|0',
    'C07|TURN_L|270', 'C07|MOVE|50', 'C07|MOVE|-1', 'WAYPOINT_1', '7|MOVE|1'])
def test_invalid(text):
    with pytest.raises(ValueError):
        parse_command(text)


def test_command_id():
    assert command_id('C07|TURN_L|60') == 'C07'
    assert command_id('WAYPOINT_1') == ''
    assert command_id('') == ''
