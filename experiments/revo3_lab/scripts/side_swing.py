"""Opt-in lateral-angle mapping; legacy opposition solvers remain unchanged."""
from lab_common import configure
configure()
import numpy as np

SIDE_DOFS = np.array([5, 9, 13, 17])


def lateral_observation(points):
    """Projected proximal/metacarpal angle, with explicit observability.

    The projected line has a pi ambiguity when flexion crosses 90 degrees.
    This convention assumes ab/adduction lies within +/-90 degrees.
    It is not a calibration of human neutral posture or a contact estimator.
    """
    raw, confidence = [], []
    for k in range(4):
        j = 5+5*k
        meta = points[j+1]-points[j]
        proximal = points[j+2]-points[j+1]
        finite = np.isfinite(meta).all() and np.isfinite(proximal).all()
        if not finite:
            raw.append(0.); confidence.append(0.); continue
        lengths = np.array([np.linalg.norm(meta), np.linalg.norm(proximal)])
        projection = np.array([np.linalg.norm(meta[1:]), np.linalg.norm(proximal[1:])])
        confidence.append(float(np.min(projection/np.maximum(lengths, 1e-12))) if lengths.min()>1e-6 else 0.)
        raw.append(-(np.arctan2(proximal[1],proximal[2])-np.arctan2(meta[1],meta[2])))
    raw = np.array(raw)
    wrapped = .5*np.arctan2(np.sin(2*raw), np.cos(2*raw))
    return raw, wrapped, np.array(confidence)


class SideMapper:
    def __init__(self, low, high, kind='observable', enter=.30, exit=.20, stale_s=.25, dt=1/30):
        self.low, self.high, self.kind = low, high, kind
        self.enter, self.exit, self.stale_s, self.dt = enter, exit, stale_s, dt
        self.previous = np.zeros(4)
        self.active = np.zeros(4,dtype=bool)
        self.age = np.zeros(4)

    def update(self, points):
        raw, wrapped, confidence = lateral_observation(points)
        if self.kind=='legacy': target = raw
        elif self.kind=='wrapped': target = wrapped
        elif self.kind=='observable':
            self.active = confidence >= np.where(self.active,self.exit,self.enter)
            self.age = np.where(self.active,0.,self.age+self.dt)
            # Hold a short ambiguous interval; then return towards neutral.
            # Joint rate/acceleration limits are applied by CommandLimiter.
            fallback = np.where(self.age<=self.stale_s,self.previous,0.)
            target = np.where(self.active,wrapped,fallback)
        else: raise ValueError('Unknown lateral mapping candidate')
        self.previous = np.clip(target,self.low,self.high)
        return self.previous.copy(), {'raw':raw,'wrapped':wrapped,'confidence':confidence,
                                     'observable':self.active.copy(),'stale_s':self.age.copy()}


def project_flexion_clearance(hand, desired):
    """Preserve lateral commands; adjust flexion to reduce original-mesh contact.

    All collisions are penalized, including the thumb and palm. This does not
    alter geometry, filtering, joint limits or the acceptance threshold.
    """
    from scipy.optimize import least_squares
    active=np.setdiff1d(np.arange(5,21),SIDE_DOFS)
    cache={}
    def evaluate(x):
        if 'x' in cache and np.array_equal(x,cache['x']):return cache['r'],cache['j']
        q=desired.copy();q[active]=x;hand.fk(q)
        cr,cj=hand.collision_penalty(0.,(-2,-1))
        r=np.r_[(x-desired[active])/.3,100*cr]
        j=np.vstack([np.eye(len(active))/.3,100*cj[:,active]])
        cache.update(x=x.copy(),r=r,j=j)
        return r,j
    x=np.clip(desired[active],hand.lo[active]+1e-7,hand.hi[active]-1e-7)
    result=least_squares(lambda x:evaluate(x)[0],x,jac=lambda x:evaluate(x)[1],
        bounds=(hand.lo[active],hand.hi[active]),max_nfev=40,ftol=1e-6,xtol=1e-6,gtol=1e-6)
    q=desired.copy();q[active]=result.x
    return q,{'converged':bool(result.success),'nfev':int(result.nfev),
              'max_flexion_correction_deg':float(np.degrees(abs(q-desired)).max())}


def retreat_clearance(hand, desired):
    """Deterministic largest-safe grid retreat, with explicit motion loss.

    Try flexion-only retreat first. If lateral targets collide even with open
    fingers, retreat lateral commands as well. Static feasibility is checked
    against unchanged mesh contacts; continuous dynamics still need evaluation.
    """
    import mujoco
    flex=np.setdiff1d(np.arange(5,21),SIDE_DOFS)
    attempts=0
    for side_scale in np.linspace(1.,0.,21):
        for flex_scale in (np.linspace(1.,0.,21) if side_scale==1 else [0.]):
            q=desired.copy();q[flex]*=flex_scale;q[SIDE_DOFS]*=side_scale
            hand.fk(q);mujoco.mj_collision(hand.model,hand.data);attempts+=1
            penetration=max((max(0.,-float(c.dist)) for c in hand.data.contact),default=0.)
            if penetration<=.00005:
                return q,{'converged':True,'nfev':attempts,'flex_scale':float(flex_scale),
                    'side_scale':float(side_scale),'static_penetration_m':penetration,
                    'max_side_correction_deg':float(np.degrees(abs(q[SIDE_DOFS]-desired[SIDE_DOFS])).max()),
                    'max_flexion_correction_deg':float(np.degrees(abs(q[flex]-desired[flex])).max())}
    raise RuntimeError('No safe neutral retreat; inspect the model instead of ignoring contacts')
