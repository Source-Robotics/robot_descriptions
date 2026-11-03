"""Build and check eight model archives and two MoveIt configuration archives."""

from pathlib import Path
import json
import ast
import tempfile
import zipfile
import xml.etree.ElementTree as ET

import collada
import mujoco

from split_motor_visuals import ROOT, MJCF, URDF


from package_layout import write_index, verify as verify_layout
from model_licensing import apply as apply_license

def expected_masses(profile, measurements):
    if profile == 'pneumatic':
        report = json.loads((ROOT/'reports/pneumatic_geometry.json').read_text())
        return {name: part['mass_kg'] for name, part in report['parts'].items()}
    from sync_model_dynamics import tool_masses
    family = 'ssg48' if profile.startswith('ssg48') else 'msg' if profile.startswith('msg') else 'flange'
    return tool_masses(family, measurements)


def build():
    destination=ROOT/'dist';destination.mkdir(exist_ok=True)
    version=ET.parse(URDF/'package.xml').findtext('version')
    result={}
    distributions=[(MJCF,'par6_mujoco.zip',True),(URDF,'par6_urdf.zip',False),
                   (ROOT/'mujoco/parol6','parol6_mujoco.zip',True),
                   (ROOT/'urdf/parol6_description','parol6_urdf.zip',False),
                   (ROOT/'mujoco/msg','msg_mujoco.zip',True),
                   (ROOT/'urdf/msg_description','msg_urdf.zip',False),
                   (ROOT/'mujoco/ssg48','ssg48_mujoco.zip',True),
                   (ROOT/'urdf/ssg48_description','ssg48_urdf.zip',False)]
    for package,filename,native in distributions:
        if not package.is_dir():raise ValueError(f'Build the missing package first: {package.name}')
        apply_license(package)
        write_index(package);verify_layout(package)
        archive=destination/filename
        with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as output:
            for path in sorted(package.rglob('*')):
                if path.is_file() and '__pycache__' not in path.parts:
                    # Filesystem checkout/build times must not change a release
                    # checksum when all packaged contents are identical.
                    info=zipfile.ZipInfo(path.relative_to(package.parent).as_posix(),date_time=(1980,1,1,0,0,0))
                    info.create_system=3;info.external_attr=0o100644<<16
                    data=path.read_bytes()
                    if path.suffix.lower() in {'.md','.xml','.urdf','.json','.obj','.dae','.mtl','.txt','.yaml','.yml','.rviz','.launch'} or path.name=='LICENSE':
                        data=data.replace(b'\r\n',b'\n')
                    output.writestr(info,data,compress_type=zipfile.ZIP_DEFLATED,compresslevel=9)
        with tempfile.TemporaryDirectory(prefix='par6_check_',dir=destination) as temporary:
            directory=Path(temporary).resolve()
            assert directory.is_relative_to(destination.resolve())
            with zipfile.ZipFile(archive) as source:
                assert source.testzip() is None
                assert all((directory/name).resolve().is_relative_to(directory) for name in source.namelist())
                source.extractall(directory)
            unpacked=directory/package.name
            verify_layout(unpacked)
            metadata=json.loads((unpacked/'sim2bot.robot.json').read_text())
            assert metadata['packageVersion']==version
            if metadata.get('category')=='gripper':
                from verify_standalone_grippers import check_package
                result[filename]={'bytes':archive.stat().st_size,**check_package(unpacked,native)}
                print(filename,result[filename],flush=True)
                continue
            assert {item['label'] for item in metadata['variants']}=={
                'Flange only','MSG - Solid jaws','MSG - Soft TPU jaws','SSG48 - Solid jaws','SSG48 - Fin-ray jaws','Pneumatic gripper'}
            assert all((unpacked/name).is_file() for name in ['README.md','docs/DYNAMICS.md','docs/DYNAMICS_AUDIT.md','docs/GRASP_FRAMES.md','docs/CONTACT_PERFORMANCE.md','docs/DH.md','docs/KINEMATICS_COMPARISON.md','docs/kinematics_comparison.svg','docs/ACTUATION.md','docs/KINEMATIC_REVIEW.md','docs/PNEUMATIC.md','docs/SSG48_CONTACTS.md'])
            for variant in metadata['variants']:
                profile=variant['id'].replace('-','_')
                entry=unpacked/variant['entry'];assert entry.is_file()
                for suffix in ['svg','png','json']:
                    assert (unpacked/f'docs/kinematics_review/{profile}.{suffix}').is_file()
                if 'preview' in variant:assert (unpacked/variant['preview']).is_file()
                if native:
                    model=mujoco.MjModel.from_xml_path(str(entry))
                    assert model.nq in [6,8]
                    measured=json.loads((ROOT/'source/dynamics/physical_measurements.json').read_text())
                    for body,mass in expected_masses(profile,measured).items():
                        assert abs(float(model.body(body).mass[0])-mass)<1e-12
                else:
                    for mesh in ET.parse(entry).findall('.//mesh'):
                        path=(unpacked/mesh.get('filename')).resolve()
                        assert path.is_relative_to(unpacked.resolve()) and path.is_file()
                        if path.suffix.lower()=='.dae':
                            document=collada.Collada(str(path))
                            for image in document.images:assert (path.parent/image.path).is_file()
                    for side in ['left','right']:
                        link=ET.parse(entry).find(f'link[@name="{side}_finger_link"]')
                        if link is not None:
                            count = len(link.findall('collision'))
                            assert count in range(1, 7) if profile == 'pneumatic' else count == 6
                    measured=json.loads((ROOT/'source/dynamics/physical_measurements.json').read_text())
                    for body,mass in expected_masses(profile,measured).items():
                        actual=float(ET.parse(entry).find(f'link[@name="{body}"]/inertial/mass').get('value'))
                        assert abs(actual-mass)<1e-12
            benches=0
            if native:
                for path in (unpacked/'bench').glob('*.xml'):
                    mujoco.MjModel.from_xml_path(str(path));benches+=1
                assert benches==8
            result[filename]={'bytes':archive.stat().st_size,'variants_checked':len(metadata['variants']),'grasp_scenes_compiled':benches,'crc_and_assets_valid':True}
        print(filename,result[filename],flush=True)
    # Config-only MoveIt packages deliberately depend on the corresponding URDF
    # package rather than copying geometry and creating another model source.
    for family in ['par6', 'parol6']:
        package=ROOT/f'moveit/{family}_moveit_config';archive=destination/f'{family}_moveit_config.zip'
        with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as output:
            for path in sorted(package.rglob('*')):
                if not path.is_file() or '__pycache__' in path.parts:continue
                info=zipfile.ZipInfo(path.relative_to(package.parent).as_posix(),date_time=(1980,1,1,0,0,0))
                info.create_system=3;info.external_attr=0o100644<<16
                output.writestr(info,path.read_bytes().replace(b'\r\n',b'\n'),compress_type=zipfile.ZIP_DEFLATED,compresslevel=9)
        with tempfile.TemporaryDirectory(prefix='moveit_check_',dir=destination) as temporary:
            directory=Path(temporary).resolve();assert directory.is_relative_to(destination.resolve())
            with zipfile.ZipFile(archive) as source:
                assert source.testzip() is None
                assert all((directory/name).resolve().is_relative_to(directory) for name in source.namelist())
                source.extractall(directory)
            unpacked=directory/package.name
            manifest=ET.parse(unpacked/'package.xml')
            assert manifest.findtext('version')==version
            assert family+'_description' in [d.text for d in manifest.findall('exec_depend')]
            assert len(list((unpacked/'config/profiles').glob('*.srdf')))==6
            for script in unpacked.rglob('*.py'):ast.parse(script.read_text())
            for path in (unpacked/'config/profiles').glob('*.json'):
                data=json.loads(path.read_text());assert data['family']==family and data['tcp']=='tcp'
                ET.parse(path.with_suffix('.srdf'))
            assert not any(p.suffix.lower() in ['.stl','.dae','.obj'] for p in unpacked.rglob('*'))
        result[archive.name]=dict(bytes=archive.stat().st_size,variants_checked=6,crc_and_assets_valid=True,
                                  dependency=family+'_description')
        print(archive.name,result[archive.name],flush=True)
    (ROOT/'reports/package_validation.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':build()
