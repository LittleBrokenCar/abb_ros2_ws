from setuptools import find_packages
from setuptools import setup

package_name = 'cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src', exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Qiao Li',
    author_email='qiao.li@zju.edu.cn',
    maintainer='Qiao Li',
    maintainer_email='qiao.li@zju.edu.cn',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD 3-Clause License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The CAM package to generate path in 3D space.'
    ),
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    package_dir={'':'src'}, 
)
