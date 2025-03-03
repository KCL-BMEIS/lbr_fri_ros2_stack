import glob

from setuptools import setup

package_name = "lbr_demos_advanced_py"

setup(
    name=package_name,
    version="2.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="Advanced Python demos for the lbr_ros2_control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "admittance_control = lbr_demos_advanced_py.admittance_control_node:main",
            "admittance_rcm_control = lbr_demos_advanced_py.admittance_rcm_control_node:main",
        ],
    },
)
