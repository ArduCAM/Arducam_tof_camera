from setuptools import setup

package_name = 'arducam_rclpy_tof_pointcloud'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='arducam',
    author_email='dennis@arducam.com',
    maintainer='arducam',
    maintainer_email='dennis@arducam.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Arducam Tof Camera Examples.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_pointcloud = ' + package_name + '.tof_pointcloud:main',
        ],
    },
)
