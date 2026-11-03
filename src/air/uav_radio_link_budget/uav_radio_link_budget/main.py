"""Estimate free-space UAV radio link range."""
from __future__ import annotations
import argparse
from importlib.resources import files
import json,math,time
from typing import Any,Dict,List
DEFAULT_INPUT=files(__package__).joinpath("sample_data","radio_profile.json")
def free_space_path_loss_db(frequency_mhz:float,distance_m:float)->float:
    return 20*math.log10(max(distance_m,1e-9))+20*math.log10(frequency_mhz*1e6)-147.55
def estimate_link(profile:Any)->tuple[Dict[str,float],List[str]]:
    if not isinstance(profile,dict): return {},["profile must be a JSON object"]
    fields=("tx_power_dbm","tx_gain_dbi","rx_gain_dbi","frequency_mhz","rx_sensitivity_dbm","fade_margin_db","additional_loss_db")
    vals={}; errors=[]
    for n in fields:
        v=profile.get(n)
        if type(v) not in (int,float) or not math.isfinite(float(v)): errors.append(f"{n} must be finite")
        else: vals[n]=float(v)
    if errors: return {},errors
    if vals["frequency_mhz"]<=0: errors.append("frequency_mhz must be positive")
    if vals["fade_margin_db"]<0 or vals["additional_loss_db"]<0: errors.append("link losses must be non-negative")
    allowed=(vals["tx_power_dbm"]+vals["tx_gain_dbi"]+vals["rx_gain_dbi"]-vals["additional_loss_db"]-vals["rx_sensitivity_dbm"]-vals["fade_margin_db"])
    if errors:return {},errors
    if allowed<=0:return {},["link budget is non-positive at zero distance"]
    distance=10**((allowed-20*math.log10(vals["frequency_mhz"]*1e6)+147.55)/20)
    return {
      "allowed_path_loss_db":round(allowed,3),
      "max_range_m":round(distance,3),
      "max_range_km":round(distance/1000,3),
      "path_loss_at_1km_db":round(free_space_path_loss_db(vals["frequency_mhz"],1000),3),
    },errors
def main(argv:List[str]|None=None)->int:
    parser=argparse.ArgumentParser(description="Estimate UAV radio link range");parser.add_argument("--input",default=DEFAULT_INPUT);parser.add_argument("--frame-delay",type=float,default=0);args=parser.parse_args(argv)
    try: profile=json.loads(open(args.input,"r",encoding="utf-8").read())
    except (OSError,TypeError,json.JSONDecodeError) as e: print(f"ERROR: {e}");return 1
    result,errors=estimate_link(profile)
    for k,v in result.items():
        print(f"{k}={v}")
        if args.frame_delay>0: time.sleep(args.frame_delay)
    for e in errors: print("ERROR:",e)
    if errors: print("FAIL: link budget is invalid");return 1
    print("PASS: free-space radio range estimated");return 0
if __name__=="__main__": raise SystemExit(main())
