from setuptools import find_packages, setup

package_name = 'tutorial_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devel',
    maintainer_email='ge65luz@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            't4_standing=tutorial_4.t4_standing:main',
            '02_one_leg_stand=tutorial_4.02_one_leg_stand:main',
            '01_standing=tutorial_4.01_standing:main',
            '03_squating=tutorial_4.03_squating:main',
            't51=tutorial_4.t51:main',
        ],
    },
)
