from setuptools import setup

package_name = "lbr_demos_py"

setup(
    name=package_name,
    version="2.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="m.huber_1994@hotmail.de",
    description="Python demos for lbr_ros2_control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_sine_overlay = lbr_demos_py.joint_sine_overlay:main",
            "joint_trajectory_client = lbr_demos_py.joint_trajectory_client:main",
            "torque_sine_overlay = lbr_demos_py.torque_sine_overlay:main",
            "wrench_sine_overlay = lbr_demos_py.wrench_sine_overlay:main",
        ],
    },
)
