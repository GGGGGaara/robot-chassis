"""Boundary checks for the requested 15 m route limit; no ROS initialization."""
import json
from common import ROOT,settings
from mission import validate_route_length

limit=settings()['navigation']['max_route_m']
checks={'configured_route_limit_15m':limit==15.,'configured_linear_limit_010':settings()['robot']['max_linear_mps']==.1}
for length in (5.1,10.,14.99,15.):
    validate_route_length(length,limit)
checks['routes_between_5_and_15m_allowed']=True
for label,length in (('over_15m',15.01),('nan',float('nan')),('negative',-1.)):
    try:validate_route_length(length,limit)
    except (ValueError,RuntimeError):checks[label+'_rejected']=True
    else:checks[label+'_rejected']=False
report={'offline_only':True,'checks':checks,'passed':all(checks.values())}
(ROOT/'results/route15_limits_test.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
