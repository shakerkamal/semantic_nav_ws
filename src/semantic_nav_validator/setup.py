from setuptools import find_packages, setup

package_name = 'semantic_nav_validator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaker',
    maintainer_email='shakerkamal18@gmail.com',
    description='Pose validator using Nav2 ComputePathToPose.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'validator_node = semantic_nav_validator.validator_node:main',
        ],
    },
)
