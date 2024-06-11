from setuptools import setup

package_name = 'py03_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'demo01_tf_static_broadcaster_py = py03_tf_broadcaster.demo01_tf_static_broadcaster_py:main',
            'demo02_tf_dynamic_broadcaster_py = py03_tf_broadcaster.demo02_tf_dynamic_broadcaster_py:main',
            'demo03_tf_point_broadcaster_py = py03_tf_broadcaster.demo03_tf_point_broadcaster_py:main'
        ],
    },
)
