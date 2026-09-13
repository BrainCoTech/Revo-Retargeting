"""Deterministic, bone-preserving MANUS-topology stress input (not a recording)."""
from lab_common import configure
configure()
import numpy as np
from synthesize_partner_input import SkeletonIK


def default_neutral(pattern):
    return [15.,30.,40.,12.] if pattern=='counter' else [15.,12.,40.,12.]


def articulated_input(base, parent, times, period_s, curl_deg, pattern='parallel'):
    ik = SkeletonIK(base, 'index')
    # Half-cosine starts at zero velocity and drives several distal hinges.
    # Index PIP extends while thumb PIP/DIP and index DIP flex.
    amplitudes = np.radians(curl_deg)*np.array([0.,0.,0.,1.,.7,0.,0.,-1.,.7])
    deltas = (1-np.cos(2*np.pi*np.asarray(times)/period_s))[:,None]*amplitudes
    if pattern=='counter':
        # Curl one thumb hinge while extending the next: change pad articulation
        # without demanding the same large increase in total distal orientation.
        deltas[:,4]=np.radians(2*curl_deg*.7)-deltas[:,4]
    elif pattern!='parallel':
        raise ValueError('Unknown synthetic curl pattern')
    if np.any(deltas < ik.lo-1e-9) or np.any(deltas > ik.hi+1e-9):
        raise ValueError('Requested curl exceeds synthetic fixture hinge bounds')
    points = np.array([ik.fk(q) for q in deltas])
    errors = []
    for child, ancestor in enumerate(parent):
        if ancestor < 0:
            continue
        lengths = np.linalg.norm(points[:,child]-points[:,ancestor],axis=1)
        errors.append(np.max(abs(lengths-np.linalg.norm(base[child]-base[ancestor]))))
    if max(errors)>1e-10:
        raise ValueError('Rubbing input changed bone lengths')
    inactive = np.setdiff1d(np.arange(len(base)), ik.thumb[1:]+ik.target[1:])
    np.testing.assert_allclose(points[:,inactive],np.broadcast_to(base[inactive],points[:,inactive].shape),atol=1e-12)
    return points, deltas, {'max_bone_length_error_m':float(max(errors)),
        'kind':'bone_preserving_synthetic_MANUS_topology','curl_pattern':pattern,
        'recorded_human_rubbing':False, 'human_surface_contact_validated':False,
        'initial_hinge_delta_deg':np.degrees(deltas[0]).tolist(),
        'hinge_delta_peak_to_peak_deg':np.degrees(np.ptp(deltas,axis=0)).tolist()}


def posture_targets(hand, base_goal, deltas, neutral_deg):
    """Explicit input-relative calibration; four distal robot hinges in radians."""
    goals=np.tile(base_goal,(len(deltas),1))
    dofs=[3,4,7,8]
    goals[:,dofs]=np.radians(neutral_deg)+deltas[:,[3,4,7,8]]-deltas[0,[3,4,7,8]]
    if np.any(goals[:,dofs]<hand.lo[dofs]) or np.any(goals[:,dofs]>hand.hi[dofs]):
        raise ValueError('Input-relative posture exceeds robot limits')
    return goals
