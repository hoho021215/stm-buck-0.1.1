"""Independent adaptive integration and steady-state check for the demo plant."""
import numpy as np
from scipy.integrate import solve_ivp
from plant import Plant

def reference(t0,t1,x,sw,load):
    if t1<=t0:
        return x
    def ode(t,x):
        il, vc = x
        vout = (vc+0.04*il)/(1+0.04/load)
        return (sw-0.10*il-vout)/470e-6, (il-vout/load)/220e-6
    return solve_ivp(ode, (t0,t1), x, atol=3e-13, rtol=1e-12, method='DOP853').y[:,-1]

def main():
    p = Plant()
    x = (0.0,0.0)
    e = 0
    for n in range(200):
        # Include duty 0/1 and both sides/exactly at the sample instant.
        duty = (0, .132, .25, .637, 1.0)[n%5]
        vin = 26.0 if n >= 90 else 30.0
        if n == 120:
            p.load = 15.0
        sampled = p.cycle(vin,duty,True)
        sample = 5e-6
        hi = duty*20e-6
        x = reference(0,min(sample, hi), x, vin, p.load)
        x = reference(min(sample, hi),sample, x, 0., p.load)
        expected = ( (x[1]+.04*x[0])/(1+.04/p.load), x[0] )
        e = max(e, *(abs(u-v) for u,v in zip(sampled,expected)))
        x = reference(sample,max(sample,hi),x,vin,p.load)
        x = reference(max(sample,hi),20e-6,x,0.,p.load)
        e = max(e,abs(x[0]-p.il),abs(x[1]-p.vc))
    print(f'200 mixed-duty cycles vs DOP853, max absolute discrepancy = {e:.3g} (V/A)')
    assert e < 1e-9
    p=Plant()
    for _ in range(15000):
        p.cycle(30,.5)
    il0, v0 = p.il,p.vc
    def averages(t,x,sw):
        i,v=x[:2]
        vo=(v+.04*i)/(1+.04/18.75)
        return [(sw-.10*i-vo)/470e-6,(i-vo/18.75)/220e-6,i,vo]
    x=np.array([il0,v0,0.,0.])
    for sw in (30.,0.):
        x=solve_ivp(averages,(0,10e-6),x,args=(sw,),atol=3e-14,rtol=1e-12,method='DOP853').y[:,-1]
    iav,vav=x[2:]/20e-6
    print(f'fixed duty 0.5 steady mean: Vout={vav:.9f} V; iL={iav:.9f} A')
    print(f'volt-second residual {abs(15-vav-.10*iav):.3g} V; charge-balance residual {abs(iav-vav/18.75):.3g} A')
    assert max(abs(15-vav-.10*iav),abs(iav-vav/18.75),abs(x[0]-il0),abs(x[1]-v0)) < 1e-8
    p.cycle(30,0,False)
    for _ in range(100):
        vo, il=p.cycle(30,0,False)
        assert il >= 0
    assert il==0 and 0<vo<v0
    print('shutdown current commutates to zero and capacitor passively discharges')

if __name__ == '__main__':
    main()
