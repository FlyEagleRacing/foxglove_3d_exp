from setuptools import find_packages, setup

package_name = 'foxglove_info_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/foxglove_publisher.launch.py']),
        ('share/' + package_name + "/boundary", ['boundary/yas_full_left.csv', 'boundary/yas_full_right.csv'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bobh',
    maintainer_email='bobh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foxglove_info_publisher_exe = foxglove_info_publisher.foxglove_info_publisher_exe:main'
        ],
    },
)
