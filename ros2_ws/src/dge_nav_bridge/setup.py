from setuptools import setup

package_name = "dge_nav_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DGE_vision",
    maintainer_email="dev@example.com",
    description="Reference navigation-side bridge for DGE auto-aim.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nav_bridge = dge_nav_bridge.nav_bridge:main",
        ],
    },
)
