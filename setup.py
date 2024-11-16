from setuptools import find_packages, setup

package_name = 'mypkg'

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
    maintainer='Abdelrahman Alhanbali',
    maintainer_email='abdelrahman.alhanbali@gmail.com',
    description='TODO: For robot system lecture',
    license='TODO: BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
