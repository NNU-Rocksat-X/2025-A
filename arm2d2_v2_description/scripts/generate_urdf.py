import xml.etree.ElementTree as ET
import sys

def _pretty_print(current, parent=None, index=-1, depth=0):
    for i, node in enumerate(current):
        _pretty_print(node, current, i, depth + 1)
    if parent is not None:
        if index == 0:
            parent.text = '\n' + ('\t' * depth)
        else:
            parent[index - 1].tail = '\n' + ('\t' * depth)
        if index == len(parent) - 1:
            current.tail = '\n' + ('\t' * (depth - 1))

def main(from_file, to_file):
    tree = ET.parse(from_file) #'/home/cyborg/Downloads/Arm-2_V1URDF.xacro'
    root = tree.getroot()

    # Remove gazebo includes
    for x in root.findall('{http://www.ros.org/wiki/xacro}include'):
        print("Removing", x.tag)
        root.remove(x)

    # add mujoco elements
    mujoco_element = ET.Element("mujoco")
    compiler_element = ET.Element("compiler", {"meshdir": "meshes", 
                                               "balanceinertia": "true",
                                               "discardvisual": "false"})
    asset_element = ET.Element("asset")
    material_element = ET.Element("material", {"name": "grid",
                                               "texture": "grid",
                                               "texrepeat": "1 1",
                                               "texuniform": "true",
                                               "reflectance": "0.2"})

    mujoco_element.append(compiler_element)
    asset_element.append(material_element)
    mujoco_element.append(asset_element)
    root.append(mujoco_element)

    _pretty_print(root)

    tree.write(to_file, encoding="utf-8")

if __name__ == "__main__":
    if len(sys.argv) == 3:
        main(sys.argv[1], sys.argv[2])
    else:
        raise Exception("Arguments: '.xacro file to generate from' 'file path to place generated urdf'")