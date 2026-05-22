from glob import glob
from setuptools import find_packages, setup

package_name = 'semantic_nav_llm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.gbnf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaker',
    maintainer_email='shakerkamal18@gmail.com',
    description='LLM navigator for semantic navigation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigator_node = semantic_nav_llm.navigator_node:main'
        ],
    },
)
