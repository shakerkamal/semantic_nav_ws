from setuptools import find_packages, setup

package_name = 'semantic_nav_executor'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Executor package bridging custom semantic navigation actions to Nav2.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'executor_node = semantic_nav_executor.executor_node:main',
        ],
    },
)
