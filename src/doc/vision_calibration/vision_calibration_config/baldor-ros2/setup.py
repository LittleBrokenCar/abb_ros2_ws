from setuptools import find_packages
from setuptools import setup

package_name = 'baldor'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages('src', exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Francisco Suarez-Ruiz',
    author_email='fsuarez6@gmail.com',
    maintainer='Francisco Suarez-Ruiz',
    maintainer_email='fsuarez6@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD 3-Clause License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The baldor package.'
    ),
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    package_dir={'':'src'}, 
)
