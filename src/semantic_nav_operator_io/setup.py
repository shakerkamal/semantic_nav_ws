from setuptools import setup

package_name = "semantic_nav_operator_io"

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
    maintainer="Md Shaker Ibna Kamal",
    maintainer_email="shakerkamal@proton.me",
    description="Operator I/O node for BT-LR M5 operator actions.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "operator_io_node = semantic_nav_operator_io.operator_io_node:main",
        ],
    },
)
