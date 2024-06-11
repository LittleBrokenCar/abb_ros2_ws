from setuptools import find_packages, setup

package_name = 'yolo_realsense_calibration'

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
    maintainer='maple',
    maintainer_email='maple@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eye_in_hand = yolo_realsense_calibration.eye_in_hand:main',
            'object_in_eye = yolo_realsense_calibration.object_in_eye:main',
            'substrate_location = yolo_realsense_calibration.substrate_location:main',
            'yolo_realsense = yolo_realsense_calibration.yolo_realsense:main',     
        ],
    },
)
