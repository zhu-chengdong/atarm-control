"""Matrix estimator guards and observability checks, without motor commands."""
from bend_cases import B,bend_config
from atarm_controller.coupled_bend import CoupledJacobian,mv,sub,length

KEY=('base','tip','map')
def window(q,p,frame):
    return {'q':tuple(q),'p':tuple(p),'deploy':0.05,'key':KEY,'first_frame':frame,'last_frame':frame+2,
        'first_s':frame*0.01,'last_s':(frame+2)*0.01,'q_spread':0.0,'pose_mad':0.0}

def model_guard(base,name,kind):
    cfg=bend_config();model=CoupledJacobian(cfg);model.new_request(KEY)
    dq=mv(B,(0.003,0.0));before=window((0,0,0),(0,0,0),1);after=window(dq,(0.0024,0.0003,0),6)
    target=dq;expected=None
    if kind=='small': dq=mv(B,(0.0001,0));after=window(dq,(0.00008,0,0),6);target=dq;expected='INSUFFICIENT_EXCITATION'
    if kind=='noise': after['p']=(0.0001,0,0);expected='NOISE_DOMINATED_TRANSLATION'
    if kind=='other_mode':
        after['q']=tuple(v+0.001 for v in after['q']);target=after['q'];expected='ACTUATION_OUTSIDE_BEND_SUBSPACE'
    if kind=='deploy': after['deploy']=0.051;expected='DEPLOY_ACTUATOR_MOVED'
    if kind=='replay': after['first_frame']=2;expected='UNPAIRED_SOURCE_FRAMES'
    if kind=='old': after['last_s']=2.0;expected='PAIR_TOO_OLD'
    if kind=='untracked': target=tuple(v+0.002 for v in target);expected='TARGET_NOT_TRACKED'
    if kind=='unstable': after['q_spread']=0.001;expected='MOTORS_NOT_SETTLED'
    if kind=='mad': after['pose_mad']=0.002;expected='NOISY_POSE_WINDOW'
    if kind=='direction': after['p']=(-0.0024,0,0);expected='DIRECTION_INCONSISTENT'
    if kind=='huge': after['p']=(0.02,0,0);expected='OBSERVED_GAIN_OUT_OF_BOUNDS'
    old=model.matrix;model.remember(before,target);accepted=model.observe(after)
    if expected:
        assert not accepted and model.events[-1]['reason']==expected
        assert model.matrix==old
    else:
        assert accepted
        assert length(sub(after['p'],mv(model.matrix,(0.003,0))))<length(sub(after['p'],mv(old,(0.003,0))))
        q2=mv(B,(0,0.003));b2=window((0,0,0),(0,0,0),11);a2=window(q2,(0.0003,0.0021,0),16)
        model.remember(b2,q2);assert model.observe(a2)
        assert model.status()['excitation_rank']==2
        assert not model.status()['full_matrix_calibration_claimed']
    base.DETAILS[name]=model.status()

def noise_pairs(base):
    model=CoupledJacobian(bend_config(noise=0.0004));model.new_request(KEY);initial=model.matrix
    for i in range(100):
        du=(0.003,0) if i%2 else (0,0.003);dq=mv(B,du)
        p=mv(initial,du);p=tuple(v+(0.0001 if (i+j)%2 else -0.0001) for j,v in enumerate(p))
        model.remember(window((0,0,0),(0,0,0),10*i+1),dq)
        assert not model.observe(window(dq,p,10*i+6))
        assert model.events[-1]['reason']=='MODEL_CONSISTENT_WITH_NOISE'
    assert model.matrix==initial and model.status()['accepted']==0
    base.DETAILS['bend_matrix_100_noise_pairs']=model.status()

def cases(base):
    result=[]
    for kind in ('accepted','small','noise','other_mode','deploy','replay','old','untracked','unstable','mad','direction','huge'):
        name='bend_matrix_'+kind
        result.append((name,lambda n=name,k=kind:model_guard(base,n,k)))
    result.append(('bend_matrix_100_noise_pairs',lambda:noise_pairs(base)))
    return result
