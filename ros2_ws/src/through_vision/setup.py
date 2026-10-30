from setuptools import setup

package_name = 'through_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rete.lin',
    maintainer_email='rete.lin@patungaero.com',
    description='Vision target tracking and test tools for the PX4 THROUGH flight mode.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_vision_target = through_vision.fake_vision_target:main',
            'through_vision = through_vision.through_vision_node:main',
        ],
    },
)
