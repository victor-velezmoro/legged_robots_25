from setuptools import find_packages, setup

package_name = 'tut_walking'

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
    maintainer='devel',
    maintainer_email='ge65luz@mytum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ex2=tut_walking.example_2_pydrake:main",
            "opc=tut_walking.ocp_lipm_2ord:main",
            "mpc=tut_walking.mpc_lipm_2ord:main",
            
        ],
    },
)
