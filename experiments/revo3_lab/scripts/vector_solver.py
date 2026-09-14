"""Geometry-derived 21-axis reference and bounded vector least-squares solvers."""
import numpy as np
from scipy.optimize import least_squares
from hand_topology import NAMES, FINGERS
from endpoint_input import EndpointTargets


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-10)


def finger_angles(points, hand):
    q = hand.q0.copy()
    for k, finger in enumerate(FINGERS[1:]):
        cmc,mcp,pip,dip,tip = [points[NAMES.index(finger+'_'+j)] for j in ['CMC','MCP','PIP','DIP','TIP']]
        meta, a, b, c = map(unit,[mcp-cmc,pip-mcp,dip-pip,tip-dip])
        start = 5+4*k
        q[start] = -(np.arctan2(a[1],a[2])-np.arctan2(meta[1],meta[2]))
        q[start+1] = np.arctan2(a[0],a[2])-np.arctan2(meta[0],meta[2])
        q[start+2] = np.arccos(np.clip(a@b,-1,1))
        q[start+3] = np.arccos(np.clip(b@c,-1,1))
    return np.clip(q,hand.lo,hand.hi)


def pair_observation(points,targets,hand):
    """All four thumb-partner observations; no task labels or commanded motion.

    Partner frame uses its distal direction and the palm transverse direction,
    never the measured thumb displacement. Axes are longitudinal/transverse/
    palmar-normal. They are skeletal proxies, not human skin-contact normals.
    """
    radial=unit(points[6]-points[21])
    frames=[];confidence=[]
    for k in range(4):
        j=5+5*k;t=unit(points[j+4]-points[j+3])
        b=-radial+t*(radial@t);length=float(np.linalg.norm(b))
        confidence.append(length)
        if length<1e-8:
            distal=unit(points[11]-points[0]);b=distal-t*(distal@t)
            if np.linalg.norm(b)<1e-8: raise ValueError('Degenerate palm frame')
            b=unit(b)  # Coordinate-only fallback; zero confidence disables pad loss.
        else:b/=length
        n=np.cross(t,b)
        frames.append(hand.basis@np.stack([t,b,n],axis=1))
    frames=np.array(frames)
    relative=targets[0]-targets[1:]
    coordinates=np.einsum('nki,nk->ni',frames,relative)
    source_tips=points[[NAMES.index(f+'_TIP') for f in FINGERS]]
    gaps=np.linalg.norm(source_tips[0]-source_tips[1:],axis=1)
    return coordinates,frames,np.asarray(confidence),gaps


def pad_pair_terms(state,coordinates,activation,cfg):
    """Input-referenced anisotropic residuals and analytical robot Jacobians."""
    from pinch_geometry import pad_tangent
    p,n,a,jp,jn,ja=state
    residual=[];jacobian=[]
    scales=np.array([cfg['pad_tangent_scale_m'],cfg.get('pad_transverse_scale_m',cfg['pad_tangent_scale_m']),cfg['pad_normal_scale_m']])
    for k in range(4):
        f=k+1;t,jt=pad_tangent(a[f],n[f],ja[f],jn[f])
        b=np.cross(n[f],t)
        jb=np.cross(jn[f].T,t).T+np.cross(n[f],jt.T).T
        axes=np.stack([t,b,n[f]])
        jaxes=np.stack([jt,jb,jn[f]])
        d=p[0]-p[f];jd=jp[0]-jp[f]
        local=axes@d
        jlocal=axes@jd+np.einsum('k,ikj->ij',d,jaxes)
        weight=np.sqrt(cfg.get('pad_pair_weight',1.)*activation[k])
        residual.append(weight*(local-coordinates[k])/scales)
        jacobian.append(weight*jlocal/scales[:,None])
        normal_weight=np.sqrt(cfg.get('pad_orientation_weight',1.)*activation[k])/cfg['pad_opposed_normal_scale']
        residual.append(normal_weight*(n[0]+n[f]));jacobian.append(normal_weight*(jn[0]+jn[f]))
    return np.concatenate(residual),np.vstack(jacobian)


class Solver:
    def __init__(self, hand, config, dt):
        if config.get('kind') == 'hybrid' and config.get('pad_pair_weight',0)>0:
            raise ValueError('Shared pad losses require the 21-axis vector solver')
        self.hand,self.config,self.dt = hand,config,dt
        self.previous = hand.q0.copy()
        self.close = False
        self.side_observer=None
        if config.get('observation_mode')=='observable':
            from side_swing import SideMapper,SIDE_DOFS
            self.side_observer=SideMapper(hand.lo[SIDE_DOFS],hand.hi[SIDE_DOFS],dt=dt)
        self.pads=None;self.source_scale=None
        self.endpoint_time = None
        self.endpoint_last_valid = None
        if config.get('pad_pair_weight',0)>0:
            from pinch_geometry import AllPadGeometry
            self.pads=AllPadGeometry(hand)

    def solve_endpoints(self, frame):
        """Retarget robot-frame endpoints without human intermediate joints.

        This uses the same optimizer, limits and filter as ``solve``. Skeleton
        posture priors are replaced by robot neutral posture regularization.
        Source-specific pad calibration remains in the legacy evaluation path.
        """
        if not isinstance(frame, EndpointTargets):
            raise TypeError('Expected EndpointTargets')
        cfg = self.config
        if cfg.get('kind') != 'vector':
            raise ValueError('Endpoint input requires the 21-axis vector solver')
        if self.pads is not None or cfg.get('observation_mode') == 'observable':
            raise ValueError('Endpoint input does not accept skeleton pad or side-observation configuration')
        for key in ('filter_tau_s', 'max_command_speed_rad_s', 'position_scale_m', 'max_dt_s'):
            value = cfg.get(key, {'position_scale_m':.05, 'max_dt_s':.1}.get(key))
            if value is None or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be positive and finite')
        for key in ('hold_s', 'posture_weight', 'temporal_weight', 'pair_weight',
                    'distal_direction_weight', 'collision_weight', 'close_weight_multiplier'):
            value = cfg.get(key, .3 if key == 'hold_s' else 0.)
            if not np.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be nonnegative and finite')
        stamp = frame.timestamp_s
        if self.endpoint_time is not None and stamp <= self.endpoint_time:
            raise ValueError('Endpoint timestamps must be strictly increasing')
        dt = self.dt if self.endpoint_time is None else stamp-self.endpoint_time
        dt = min(dt, cfg.get('max_dt_s', .1))
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Endpoint dt must be positive and finite')
        self.endpoint_time = stamp
        self.dt = dt
        if not frame.valid.any():
            age = np.inf if self.endpoint_last_valid is None else stamp-self.endpoint_last_valid
            status = 'hold' if age <= cfg.get('hold_s', .3) else 'return_neutral'
            if status == 'return_neutral':
                alpha = 1-np.exp(-self.dt/cfg['filter_tau_s'])
                step = np.clip(alpha*(self.hand.q0-self.previous),
                               -cfg['max_command_speed_rad_s']*self.dt,
                               cfg['max_command_speed_rad_s']*self.dt)
                self.previous = np.clip(self.previous+step,self.hand.lo,self.hand.hi)
            self.close = False
            return self.previous.copy(), {'status':status,'nfev':0,'converged':False,
                'ik_solution':self.previous.tolist(),'valid':frame.valid.tolist(),
                'limiter_dt_s':self.dt,'pair_activation':[0.]*4}
        self.endpoint_last_valid = stamp
        q, detail = self._solve(None, frame.positions_m, frame)
        detail.update(status='tracking' if frame.valid.all() else 'partial_tracking',
                      valid=frame.valid.tolist(), limiter_dt_s=self.dt)
        return q, detail

    def solve(self, points, target):
        """Compatibility entry point for frozen MANUS skeleton regressions."""
        return self._solve(points, target)

    def _solve(self, points, target, endpoint=None):
        hand,cfg = self.hand,self.config
        prior = finger_angles(points,hand) if endpoint is None else hand.q0.copy()
        side_diag=None
        if self.side_observer is not None and endpoint is None:
            from side_swing import SIDE_DOFS
            prior[SIDE_DOFS],side_diag=self.side_observer.update(points)
            # Remove observed abduction before measuring MCP flexion. This also
            # avoids the >90 degree projected-angle branch changing side signs.
            for k,side in enumerate(prior[SIDE_DOFS]):
                j=5+5*k;meta=unit(points[j+1]-points[j]);v=unit(points[j+2]-points[j+1])
                den=np.cos(side)*v[2]-np.sin(side)*v[1]
                flex=np.arctan2(v[0],den)-np.arctan2(meta[0],meta[2])
                prior[6+4*k]=np.clip(flex,hand.lo[6+4*k],hand.hi[6+4*k])
        gap = (np.linalg.norm(points[NAMES.index('Thumb_TIP')]-points[NAMES.index('Index_TIP')])
               if endpoint is None else
               np.linalg.norm(target[0]-target[1]) if endpoint.valid[:2].all() else np.inf)
        if self.close and gap > cfg['close_exit_m']:
            self.close = False
        elif not self.close and gap < cfg['close_enter_m']:
            self.close = True
        pair_weight = cfg['pair_weight'] * (cfg['close_weight_multiplier'] if self.close else 1.)
        scale = cfg.get('position_scale_m',.05)
        temporal = cfg['temporal_weight']
        posture = cfg['posture_weight']
        is_baseline = cfg['kind'] == 'hybrid'
        active = np.arange(5) if is_baseline else np.arange(21)
        previous = self.previous.copy()
        fixed = prior.copy()
        posture_weights=np.full(16,posture)
        if side_diag is not None:
            posture_weights[::4]=cfg.get('side_posture_weight',posture)*np.where(side_diag['observable'],1.,cfg.get('unobservable_side_weight',.05))
        activation=np.zeros(4);coordinates=None;raw_coordinates=None;source_frames=None
        if self.pads is not None:
            coordinates,source_frames,frame_confidence,gaps=pair_observation(points,target,hand)
            raw_coordinates=coordinates.copy()
            u=np.clip((cfg['close_exit_m']-gaps)/(cfg['close_exit_m']-cfg['close_enter_m']),0.,1.)
            activation=u*u*(3-2*u)*np.clip(frame_confidence/.3,0.,1.)
            if self.source_scale is None:self.source_scale=hand.robot_width/np.linalg.norm(points[6]-points[21])
            front=np.clip(coordinates[:,2]/(cfg.get('pad_front_transition_m',.002)*self.source_scale),0.,1.)
            activation*=front*front*(3-2*front)
            # Only normal clearance is calibrated for the differing surface
            # geometries. Tangential components remain the observed vectors.
            coordinates=coordinates.copy()
            coordinates[:,2]=np.maximum(0.,coordinates[:,2]-cfg.get('source_surface_clearance_m',.004)*self.source_scale)-cfg.get('pad_preload_m',.00035)*activation
        tip_weights=np.ones(5)
        if endpoint is not None:
            tip_weights = endpoint.valid.astype(float)
            posture_weights = np.full(21, posture)
        if self.pads is not None:
            involvement=np.r_[activation.max(),activation]
            tip_weights=1-involvement*(1-cfg.get('near_tip_weight',.15))
        cache = {}

        def evaluate(x):
            if 'x' in cache and np.array_equal(cache['x'],x):
                return cache['r'],cache['j']
            q = fixed.copy() if is_baseline else np.zeros(21)
            q[active] = x
            tips,_,jac = hand.fk(q,jac=True)
            delta = np.sqrt(tip_weights)[:,None]*(tips-target)/scale
            if is_baseline:
                residual = [delta[0], np.sqrt(temporal)*(x-previous[active])]
                jacobian = [jac[0][:,active]/scale, np.sqrt(temporal)*np.eye(5)]
            else:
                pair_delta = ((tips[1:]-tips[0])-(target[1:]-target[0]))/scale
                pair_weights=(cfg['pair_weight']*(1+activation*(cfg['close_weight_multiplier']-1)) if self.pads is not None else pair_weight)*(1-activation)
                if endpoint is not None:
                    pair_weights = pair_weights*(endpoint.valid[0] & endpoint.valid[1:])
                posture_slice = slice(5,None) if endpoint is None else slice(None)
                residual = [delta.ravel(), (np.sqrt(pair_weights)[:,None]*pair_delta).ravel(),
                            np.sqrt(temporal)*(x-previous), np.sqrt(posture_weights)*(x[posture_slice]-prior[posture_slice])]
                jacobian = [(np.sqrt(tip_weights)[:,None,None]*jac).reshape(15,21)/scale,
                            (np.sqrt(pair_weights)[:,None,None]*(jac[1:]-jac[0])).reshape(12,21)/scale,
                            np.sqrt(temporal)*np.eye(21),np.sqrt(posture_weights)[:,None]*np.eye(21)[posture_slice]]
                if endpoint is not None and cfg.get('distal_direction_weight',0.) and endpoint.directions is not None:
                    axes, axis_jacobian = hand.distal_directions()
                    weights = np.sqrt(cfg.get('distal_direction_weight',0.))*endpoint.direction_valid
                    residual.append((weights[:,None]*(axes-endpoint.directions)).ravel())
                    jacobian.append((weights[:,None,None]*axis_jacobian).reshape(15,21))
                if self.pads is not None:
                    state=self.pads.evaluate()
                    pr,pj=pad_pair_terms(state,coordinates,activation,cfg)
                    residual.append(pr);jacobian.append(pj)
                    direction_weight=np.sqrt(cfg.get('distal_direction_weight',0.))
                    if direction_weight:
                        source_axes=unit(points[[NAMES.index(f+'_TIP') for f in FINGERS]]-points[[NAMES.index(f+'_DIP') for f in FINGERS]])@hand.basis.T
                        residual.append(direction_weight*(state[2]-source_axes).ravel())
                        jacobian.append(direction_weight*state[5].reshape(15,21))
                collision_weight = cfg.get('collision_weight',0.)
                if collision_weight:
                    if self.pads is None:
                        cr,cj = hand.collision_penalty(cfg.get('allowed_target_penetration_m',0.0005))
                    else:
                        from revo3_model import Hand
                        allowed={frozenset([0,k+1]):cfg.get('allowed_target_penetration_m',.0005)*activation[k] for k in range(4)}
                        cr,cj=Hand.collision_penalty(hand,allowed_pairs=allowed)
                    residual.append(np.sqrt(collision_weight)*cr)
                    jacobian.append(np.sqrt(collision_weight)*cj)
            r,j = np.concatenate(residual),np.vstack(jacobian)
            cache.update(x=x.copy(),r=r,j=j)
            return r,j
        initial = np.clip(previous[active],hand.lo[active]+1e-7,hand.hi[active]-1e-7)
        result = least_squares(lambda x:evaluate(x)[0],initial,jac=lambda x:evaluate(x)[1],
                               bounds=(hand.lo[active],hand.hi[active]),max_nfev=cfg['max_nfev'],
                               ftol=1e-5,xtol=1e-5,gtol=1e-5)
        q = fixed.copy() if is_baseline else np.zeros(21)
        q[active] = result.x
        # Identical one-pole smoothing, rate limiting and joint bounds for every candidate.
        alpha = 1-np.exp(-self.dt/cfg['filter_tau_s'])
        step = np.clip(alpha*(q-previous),-cfg['max_command_speed_rad_s']*self.dt,cfg['max_command_speed_rad_s']*self.dt)
        self.previous = np.clip(previous+step,hand.lo,hand.hi)
        detail={'nfev':result.nfev,'converged':result.success,'close_proxy':self.close,'ik_solution':q.tolist(),
                'pair_activation':activation.tolist()}
        if side_diag is not None:
            detail.update(side_reference_rad=prior[[5,9,13,17]].tolist(),side_observable=side_diag['observable'].tolist())
        if coordinates is not None:
            detail['input_pair_coordinates_m']=raw_coordinates.tolist()
            detail['target_pair_coordinates_m']=coordinates.tolist()
        return self.previous.copy(),detail
