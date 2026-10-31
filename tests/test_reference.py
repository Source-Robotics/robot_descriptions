import json

import numpy as np
import pytest

from model_reference import ROOT,load_reference_robot

CASES=json.loads((ROOT/'source/dynamics/rcb_reference_checks.json').read_text())['cases']


@pytest.mark.parametrize('case',CASES,ids=lambda case:case['tool_name'])
def test_portable_reference_matches_recorded_live_runtime(case):
    model=load_reference_robot(case['tool_name']);q=case['q_rad']
    np.testing.assert_allclose(model.fkine(q).A,case['tcp_pose'],atol=1e-14,rtol=0)
    np.testing.assert_allclose(model.inertia(q),case['mass_matrix'],atol=1e-14,rtol=1e-12)
    np.testing.assert_allclose(model.gravload(q),case['gravity_torque_nm'],atol=1e-12,rtol=0)


def test_unknown_reference_is_explicit_error():
    with pytest.raises(ValueError,match='Unknown tool'):load_reference_robot('unknown')
