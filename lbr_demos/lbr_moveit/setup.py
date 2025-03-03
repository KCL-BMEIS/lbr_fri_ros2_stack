from setuptools import find_packages, setup

package_name = "lbr_moveit"

setup(
    name=package_name,
    version="2.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/forward_keyboard.yaml"]),
        ("share/" + package_name + "/launch", ["launch/keyboard_driver.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="MoveIt demos for the LBRs.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "forward_keyboard = scripts.forward_keyboard:main",
        ],
    },
)
