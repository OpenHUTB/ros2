import argparse
import json
import subprocess
import sys
import zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','collect','train','evaluate','report','test','doctor','build','demo']);a,extra=p.parse_known_args()
    if a.stage=='prepare':
        with zipfile.ZipFile(ROOT/'assets/landing_corpus.zip') as archive:
            for item in archive.infolist():
                assert not Path(item.filename).is_absolute() and '..' not in Path(item.filename).parts
            archive.extractall(ROOT)
        return
    if a.stage in ('doctor','build'):
        from course_environment import doctor,build
        doctor(write=True) if a.stage=='doctor' else build('landing_assessment',ROOT)
        return
    scripts={'collect':['-m','landing_assessment.simulation'],'train':['-m','landing_assessment.learning','train'],
             'evaluate':['-m','landing_assessment.learning','evaluate'],'report':['-m','landing_assessment.report'],
             'test':['-m','unittest','discover','-s','tests','-v']}
    if a.stage in scripts:
        subprocess.run([sys.executable]+scripts[a.stage]+extra,cwd=ROOT,check=True);return
    if not (ROOT/'data/raw/manifest.json').exists():subprocess.run([sys.executable,str(ROOT/'main.py'),'prepare'],check=True)
    if not (ROOT/'install/setup.bash').exists():subprocess.run([sys.executable,str(ROOT/'main.py'),'build'],check=True)
    subprocess.run(['bash',str(ROOT/'demo.sh')]+extra,cwd=ROOT,check=True)

if __name__=='__main__':main()
