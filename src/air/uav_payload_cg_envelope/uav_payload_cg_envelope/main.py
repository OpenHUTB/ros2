"""Compute combined UAV center of gravity and check payload limits."""
from __future__ import annotations
import argparse
from importlib.resources import files
import json,math,time
from typing import Any,Dict,List
DEFAULT_INPUT=files(__package__).joinpath("sample_data","cg_profile.json")
def _number(v:Any,name:str,errors:List[str],positive=False)->float:
 if type(v) not in (int,float) or not math.isfinite(float(v)): errors.append(f"{name} must be finite"); return 0.0
 n=float(v)
 if positive and n<=0: errors.append(f"{name} must be positive")
 return n
def _vector(v:Any,name:str,errors:List[str])->tuple[float,float,float]:
 if not isinstance(v,list) or len(v)!=3 or any(type(x) not in (int,float) or not math.isfinite(float(x)) for x in v): errors.append(f"{name} must contain three finite numbers");return (0,0,0)
 return tuple(float(x) for x in v)
def check_cg(profile:Any)->tuple[Dict[str,Any],List[str]]:
 if not isinstance(profile,dict):return {},["profile must be a JSON object"]
 errors=[]; empty=profile.get("empty",{}); limits=profile.get("limits",{}); items=profile.get("payloads")
 if not isinstance(empty,dict) or not isinstance(limits,dict):return {},["empty and limits must be objects"]
 mass=_number(empty.get("mass_kg"),"empty.mass_kg",errors,True); cg=_vector(empty.get("cg_m"),"empty.cg_m",errors)
 xmin=_number(limits.get("cg_x_min_m"),"limits.cg_x_min_m",errors); xmax=_number(limits.get("cg_x_max_m"),"limits.cg_x_max_m",errors); lateral=_number(limits.get("lateral_max_m"),"limits.lateral_max_m",errors,True)
 if xmin>=xmax: errors.append("cg_x_min_m must be less than cg_x_max_m")
 if not isinstance(items,list): errors.append("payloads must be a list");items=[]
 moments=[mass*cg[0],mass*cg[1],mass*cg[2]]; total=mass; rows=[]
 for i,item in enumerate(items):
  if not isinstance(item,dict): errors.append(f"payloads[{i}] must be an object");continue
  m=_number(item.get("mass_kg"),f"payloads[{i}].mass_kg",errors,True); pos=_vector(item.get("cg_m"),f"payloads[{i}].cg_m",errors); total+=m
  for a in range(3):moments[a]+=m*pos[a]
  rows.append({"name":str(item.get("name",f"payload_{i}")),"mass_kg":m})
 if errors:return {},errors
 combined=(moments[0]/total,moments[1]/total,moments[2]/total)
 front_margin=combined[0]-xmin; aft_margin=xmax-combined[0]; warnings=[]; feasible=front_margin>=0 and aft_margin>=0 and abs(combined[1])<=lateral
 if feasible and min(front_margin,aft_margin)<0.005:warnings.append("longitudinal CG margin below 5 mm")
 return {"total_mass_kg":round(total,4),"cg_x_m":round(combined[0],5),"cg_y_m":round(combined[1],5),"cg_z_m":round(combined[2],5),"front_margin_m":round(front_margin,5),"aft_margin_m":round(aft_margin,5),"lateral_abs_m":round(abs(combined[1]),5),"feasible":feasible,"warnings":warnings,"payloads":rows},errors
def main(argv:List[str]|None=None)->int:
 p=argparse.ArgumentParser(description="Check UAV payload center of gravity");p.add_argument("--input",default=DEFAULT_INPUT);p.add_argument("--frame-delay",type=float,default=0);a=p.parse_args(argv)
 try:profile=json.loads(open(a.input,"r",encoding="utf-8").read())
 except (OSError,TypeError,json.JSONDecodeError) as e:print(f"ERROR: {e}");return 1
 r,e=check_cg(profile)
 for k,v in r.items():
  if k!="payloads":
   print(f"{k}={v}")
  if a.frame_delay>0:time.sleep(a.frame_delay)
 for x in e:print("ERROR:",x)
 for w in r.get("warnings",[]):print("WARNING:",w)
 if e or not r.get("feasible",False):print("FAIL: payload CG is outside the configured envelope");return 1
 print("PASS: payload CG remains inside the configured envelope");return 0
if __name__=="__main__":raise SystemExit(main())
