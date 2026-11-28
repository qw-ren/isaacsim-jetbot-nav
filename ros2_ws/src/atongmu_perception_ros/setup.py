from glob import glob
from setuptools import setup

package_name = "atongmu_perception_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="atongmu perception",
    maintainer_email="todo@example.com",
    description="阿童木课题视觉感知模块的 ROS2 节点",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_node = atongmu_perception_ros.perception_node:main",
            "planar_map_node = atongmu_perception_ros.planar_map_node:main",
        ],
    },
)
