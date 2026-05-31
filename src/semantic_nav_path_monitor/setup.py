from setuptools import setup

package_name = 'semantic_nav_path_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaker',
    maintainer_email='shakerkamal18@gmail.com',
    description='Plan/costmap intersection monitor for semantic navigation recovery triggers.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plan_intersection_monitor = semantic_nav_path_monitor.plan_intersection_monitor:main',
        ],
    },
)
