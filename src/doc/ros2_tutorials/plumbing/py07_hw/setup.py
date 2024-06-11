from setuptools import setup

package_name = 'py07_hw'

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
            'exer01_pub_sub_py = py07_hw.exer01_pub_sub_py:main',
            'exer02_server_py = py07_hw.exer02_server_py:main',
            'exer03_client_py = py07_hw.exer03_client_py:main',
            'exer04_action_server_py = py07_hw.exer04_action_server_py:main',
            'exer05_action_client_py = py07_hw.exer05_action_client_py:main'
        ],
    },
)
