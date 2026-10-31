from pathlib import Path
import subprocess
import sys

from model_reference import ROOT


def test_every_tool_imports_with_application_and_runtime_imports_blocked():
    script='''
import importlib,importlib.abc,pathlib,sys
sys.path.insert(0,sys.argv[1])
class BlockRuntime(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname.split('.')[0] in {'par6_motion','robots','roboticstoolbox','pinocchio'}:
            raise RuntimeError('Forbidden external runtime import: '+fullname)
sys.meta_path.insert(0,BlockRuntime())
for file in pathlib.Path(sys.argv[1]).glob('*.py'):
    importlib.import_module(file.stem)
'''
    subprocess.run([sys.executable,'-I','-B','-c',script,str(ROOT/'tools')],cwd=ROOT,check=True)
