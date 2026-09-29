"""The protocol lives in four places: protocol/vector.dbc, the firmware's C headers,
vector_hw's C++ and leg_config.py. Numbers and names have to agree between them."""
import importlib.util
import os
import re
import sys
import types

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), *['..'] * 5))
FW = os.path.join(REPO, 'firmware', 'common')


def c_enums(path):
    """NAME -> value for every enum in a C header."""
    text = re.sub(r'/\*.*?\*/', '', open(path).read(), flags=re.S)
    out = {}
    for body in re.findall(r'enum\s*\w*\s*\{(.*?)\}', text, re.S):
        value = -1
        for item in filter(None, (i.strip() for i in body.split(','))):
            name, _, expr = item.partition('=')
            name = name.strip()
            if expr.strip():
                value = eval(re.sub(r'(?<=[0-9a-fA-F])u\b', '', expr.strip()), {}, dict(out))
            else:
                value += 1
            out[name] = value
    return out


def dbc_values():
    """(frame id, signal) -> {value: NAME} from the VAL_ lines."""
    out = {}
    for fid, sig, rest in re.findall(r'^VAL_ (\d+) (\w+) (.*);', open(os.path.join(REPO, 'protocol', 'vector.dbc')).read(), re.M):
        out[(int(fid), sig)] = {int(v): n for v, n in re.findall(r'(\d+) "(\w+)"', rest)}
    return out


def leg_config():
    sys.modules.setdefault('can', types.ModuleType('can'))  # python-can is only needed on a bus
    path = os.path.join(REPO, 'software', 'ros2_ws', 'src', 'vector_hw', 'scripts', 'leg_config.py')
    spec = importlib.util.spec_from_file_location('leg_config', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


C = {**c_enums(os.path.join(FW, 'vector_can.h')), **c_enums(os.path.join(FW, 'boot.h'))}
DBC = dbc_values()
LC = leg_config()


def prefixed(prefix):
    return {v: k[len(prefix):] for k, v in C.items() if k.startswith(prefix)}


def test_functions():
    assert LC.LEG_STATUS == C['CAN_LEG_STATUS']
    assert LC.LEG_CONFIG == C['CAN_LEG_CONFIG']
    assert LC.LEG_REPLY == C['CAN_LEG_REPLY']
    assert LC.CAN_BOOT == C['CAN_BOOT']
    assert LC.CAN_BOOT_REPLY == C['CAN_BOOT_REPLY']


def test_leg_states():
    c = prefixed('LEG_')
    assert [c[i].lower() for i in range(len(c))] == LC.STATES
    for fid in range(0x31, 0x37):  # LEG_L1_STATUS .. LEG_R3_STATUS
        assert {k: v.lower() for k, v in DBC[(fid, 'state')].items()} == dict(enumerate(LC.STATES))
    cpp = open(os.path.join(REPO, 'software', 'ros2_ws', 'src', 'vector_hw', 'src', 'protocol.cpp')).read()
    names = re.search(r'state_name.*?\{(.*?)\}', cpp, re.S).group(1)
    assert re.findall(r'"(\w+)"', names) == LC.STATES


def test_faults():
    c = {v.bit_length() - 1: k[len('FAULT_'):].lower() for k, v in C.items() if k.startswith('FAULT_')}
    assert [c[i] for i in range(8)] == LC.FAULTS
    cpp = open(os.path.join(REPO, 'software', 'ros2_ws', 'src', 'vector_hw', 'src', 'protocol.cpp')).read()
    names = re.search(r'fault_names.*?\{(.*?)\}', cpp, re.S).group(1)
    assert re.findall(r'"(\w+)"', names) == LC.FAULTS


def test_config_ops_and_statuses():
    assert (LC.OP_READ, LC.OP_WRITE, LC.OP_SAVE, LC.OP_CALIBRATE, LC.OP_DEFAULTS, LC.OP_SELFTEST) == (
        C['OP_READ'], C['OP_WRITE'], C['OP_SAVE'], C['OP_CALIBRATE'], C['OP_DEFAULTS'], C['OP_SELFTEST'])
    assert (LC.ST_OK, LC.ST_CAL_RESULT, LC.ST_CAL_FAILED, LC.ST_CAL_DONE) == (
        C['ST_OK'], C['ST_CAL_RESULT'], C['ST_CAL_FAILED'], C['ST_CAL_DONE'])
    assert (LC.ST_TEST_PASS, LC.ST_TEST_FAIL, LC.ST_TEST_DONE) == (C['ST_TEST_PASS'], C['ST_TEST_FAIL'], C['ST_TEST_DONE'])
    st = prefixed('ST_')
    assert len(LC.STATUS_NAMES) == len(st)
    for fid in range(0x51, 0x57):
        assert DBC[(fid, 'op')] == {v: k for k, v in {n: C['OP_' + n] for n in
                                   ('READ', 'WRITE', 'SAVE', 'CALIBRATE', 'DEFAULTS', 'SELFTEST')}.items()}
    for fid in range(0x61, 0x67):
        assert DBC[(fid, 'status')] == st


def test_keys():
    joint_keys = {'mid_mv': 'WIPER_MID', 'slope': 'WIPER_SLOPE', 'center_us': 'CENTER_US', 'direction': 'DIRECTION',
                  'us_per_deg': 'US_PER_DEG', 'min_deg': 'MIN_DEG', 'max_deg': 'MAX_DEG', 'fit_err_mv': 'FIT_ERR',
                  'calibrated': 'CALIBRATED'}
    assert {C['KEY_' + v]: k for k, v in joint_keys.items()} == {k: name for k, (name, _) in LC.KEYS.items()}
    leg_keys = {'version': 'VERSION', 'reset_cause': 'RESET_CAUSE', 'can_errors': 'CAN_ERRORS', 'uptime_s': 'UPTIME',
                'vbat_gain': 'VBAT_GAIN', 'rail_gain': 'RAIL_GAIN', 'i_zero_ma': 'I_ZERO', 'id_straps': 'ID_STRAPS'}
    assert {k: C['KEY_' + v] for k, v in leg_keys.items()} == LC.LEG_KEY
    resets = {v.bit_length() - 1: k[len('RESET_'):].lower() for k, v in C.items() if k.startswith('RESET_')}
    assert [resets[i] for i in range(len(resets))] == LC.RESETS


def test_selftest_items():
    items = {v: k for k, v in C.items() if k.startswith('TEST_')}
    assert sorted(items) == sorted(LC.TESTS)


def test_bootloader():
    assert (LC.B_ENTER, LC.B_INFO, LC.B_ERASE, LC.B_DATA, LC.B_DONE, LC.B_RUN) == (
        C['BOOT_ENTER'], C['BOOT_INFO'], C['BOOT_ERASE'], C['BOOT_DATA'], C['BOOT_DONE'], C['BOOT_RUN'])
    assert (LC.B_OK, LC.B_BUSY) == (C['BOOT_OK'], C['BOOT_BUSY'])
    names = ('OK', 'BAD_OP', 'BAD_SEQ', 'FLASH_ERR', 'BAD_CRC', 'TOO_BIG', 'NO_IMAGE', 'BUSY')
    status = {C['BOOT_' + n]: n for n in names}
    assert len(LC.BOOT_STATUS) == len(status)
    assert LC.BOOT_STATUS[C['BOOT_BAD_CRC']] == 'bad crc' and LC.BOOT_STATUS[C['BOOT_BUSY']] == 'busy'
    for fid in range(0x81, 0x87):
        assert DBC[(fid, 'status')] == status
    for fid in range(0x71, 0x77):
        assert DBC[(fid, 'op')] == {C['BOOT_' + n]: n for n in ('ENTER', 'INFO', 'ERASE', 'DATA', 'DONE', 'RUN')}


def test_power():
    cpp = open(os.path.join(REPO, 'software', 'ros2_ws', 'src', 'vector_hw', 'src', 'protocol.cpp')).read()
    states = {v: k[len('POWER_'):].lower() for k, v in C.items() if k.startswith('POWER_')}
    assert [states[i] for i in range(len(states))] == LC.POWER_STATES
    assert {k: v.lower() for k, v in DBC[(0x47, 'state')].items()} == dict(enumerate(LC.POWER_STATES))
    assert re.findall(r'"(\w+)"', re.search(r'power_state_name.*?\{(.*?)\}', cpp, re.S).group(1)) == LC.POWER_STATES
    faults = {v.bit_length() - 1: k[len('PWR_FAULT_'):].lower() for k, v in C.items() if k.startswith('PWR_FAULT_')}
    assert [faults[i] for i in range(8)] == LC.POWER_FAULTS
    assert re.findall(r'"(\w+)"', re.search(r'power_fault_names.*?\{(.*?)\}', cpp, re.S).group(1)) == LC.POWER_FAULTS
    assert int(re.search(r'#define POWER_NODE (\d+)', open(os.path.join(FW, 'vector_can.h')).read()).group(1)) == LC.POWER
    keys = {'capacity_mah': 'CAPACITY', 'charge_ma': 'CHARGE_MA', 'charge_mv': 'CHARGE_MV', 'input_ma': 'INPUT_MA',
            'low_mv': 'LOW_MV', 'sides': 'SIDES', 'soc_permille': 'SOC'}
    assert {k: C['PKEY_' + v] for k, v in keys.items()} == LC.POWER_KEYS
    assert LC.PKEY_BMS_MEM == C['PKEY_BMS_MEM']
    assert (LC.POWER_STATE, LC.POWER_CELLS, LC.POWER_DETAIL) == (
        C['CAN_POWER_STATE'], C['CAN_POWER_CELLS'], C['CAN_POWER_DETAIL'])
