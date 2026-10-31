"""Keep jaw regeneration independent of previously generated contact assets."""

import xml.etree.ElementTree as ET


def reset_contacts(root, mesh_paths=None):
    """Restore a temporary single collision mesh before selecting a new profile."""
    for parent in root.iter():
        for child in list(parent):
            if child.get('name', '').startswith('contact_'):
                parent.remove(child)
    for side in ['left', 'right']:
        if root.tag == 'mujoco':
            geom = root.find(f'.//body[@name="{side}_finger_link"]/geom')
            for key in ['contype', 'conaffinity', 'mass']:
                geom.attrib.pop(key, None)
        else:
            link = root.find(f'link[@name="{side}_finger_link"]')
            for collision in list(link.findall('collision')):
                link.remove(collision)
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'mesh', filename=mesh_paths[side])


def refresh_contacts():
    from build_gripper_contacts import build
    from build_grasp_benches import build as build_benches
    build()
    from sync_model_dynamics import synchronize
    synchronize()
    build_benches()
