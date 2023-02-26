import xml.etree.ElementTree as ET

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

def main():
    tree = ET.parse('/home/cyborg/Downloads/Arm-2_V1URDF.xacro')
    root = tree.getroot()

    # add silver material

    # add world link
    # add world joint

    # change package name

if __name__ == "__main__":
    main()