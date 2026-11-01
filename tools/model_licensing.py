"""Ship CC-BY-4.0 model notices and preserve Apache-2.0 support-code licensing."""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
MODEL_LICENSE='CC-BY-4.0'
CODE_LICENSE='Apache-2.0'


def apply(package):
    package=Path(package)
    docs=package/'docs';docs.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(ROOT/'LICENSE',package/'LICENSE')
    path=package/'sim2bot.robot.json'
    data=json.loads(path.read_text(encoding='utf-8'))
    data['license']=MODEL_LICENSE
    support=(package/'package.xml').is_file()
    if support:data['codeLicense']=CODE_LICENSE
    path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    code=''
    if support:
        shutil.copyfile(ROOT/'LICENSES/Apache-2.0.txt',docs/'LICENSE-CODE.txt')
        tree=ET.parse(package/'package.xml');root=tree.getroot()
        for item in root.findall('license'):root.remove(item)
        index=list(root).index(root.find('maintainer'))+1
        for offset,name in enumerate([MODEL_LICENSE,CODE_LICENSE]):
            item=ET.Element('license');item.text=name;root.insert(index+offset,item)
        ET.indent(tree,space='  ');tree.write(package/'package.xml',encoding='utf-8',xml_declaration=True)
        code='\nROS launch/support scripts and CMake code are Apache-2.0; see [LICENSE-CODE.txt](LICENSE-CODE.txt).\n'
    (docs/'ATTRIBUTION.md').write_text('''# Model license and attribution

Copyright 2026 Source Robotics.

The robot/gripper descriptions, geometry, textures, physical data and
documentation in this package are licensed under **CC-BY-4.0**.
See the [full model license](../LICENSE).

Source: [Source Robotics robot_descriptions](https://github.com/Source-Robotics/robot_descriptions).
License: [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/).

When sharing the models or adaptations, credit Source Robotics, link to the
source and license, retain supplied notices, and indicate your changes.
For example:

> Robot models by Source Robotics, https://github.com/Source-Robotics/robot_descriptions,
> CC BY 4.0, https://creativecommons.org/licenses/by/4.0/.
> Changes: describe your modifications, or state that none were made.
'''+code,encoding='utf-8')
    path=package/'README.md';text=path.read_text(encoding='utf-8')
    if '## License and attribution' not in text:
        text+='\n## License and attribution\n\nModels and assets: **CC BY 4.0**. See [attribution and license scope](docs/ATTRIBUTION.md).\n'
    path.write_text(text,encoding='utf-8')


def apply_all():
    for group in ['mujoco','urdf']:
        for package in (ROOT/group).iterdir():
            if (package/'sim2bot.robot.json').is_file():apply(package)
