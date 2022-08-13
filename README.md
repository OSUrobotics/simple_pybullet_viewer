# simple_pybullet_viewer
Script that can be used to view a robot and/or objects in a pybullet enviroment.

Finds urdf and obj files that are in the same lower directory as the script.

Only officially supported in linux enviroments:
- Ubuntu 18.04 +
- WSLg

## Requirements:
- Python 3.6+
- PyBullet

## Usage:

Install PyBullet:

```console
pip3 install pybullet
```

Navigate to the directory that you want the script to be located.

```console,
wget https://github.com/OSUrobotics/simple_pybullet_viewer/raw/main/viewer_tool.py
```

```console,
python3 viewer_tool.py
```
