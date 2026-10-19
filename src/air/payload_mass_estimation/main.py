import argparse,subprocess,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','collect','train','evaluate','test','doctor','build','demo']);a=p.parse_args()
    if a.stage=='prepare':
        with zipfile.ZipFile(ROOT/'assets/flights.zip') as z:
            for info in z.infolist():
                assert not Path(info.filename).is_absolute() and '..' not in Path(info.filename).parts
            z.extractall(ROOT)
    elif a.stage in ['doctor','build']:
        from course_environment import doctor,build
        doctor(write=True) if a.stage=='doctor' else build('payload_mass_estimation',ROOT)
    elif a.stage=='test':subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-v'],cwd=ROOT,check=True)
    elif a.stage=='demo':subprocess.run(['bash',str(ROOT/'demo.sh')],cwd=ROOT,check=True)
    else:subprocess.run([sys.executable,'-m','payload_mass_estimation.experiment',a.stage],cwd=ROOT,check=True)
if __name__=='__main__':main()
