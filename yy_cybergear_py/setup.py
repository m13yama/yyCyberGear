from setuptools import find_packages, setup

package_name = 'yy_cybergear_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='yuki yamamoto',
    maintainer_email='yuki8mamo10.hu@gmail.com',
    description='Python implementation of the CyberGear SocketCAN driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yy_cybergear_py_exmp_get_mcu_id = '
            'yy_cybergear_py.example.exmp_get_mcu_id:main',
            'yy_cybergear_py_exmp_speed_constant = '
            'yy_cybergear_py.example.exmp_speed_constant:main',
        ],
    },
)
