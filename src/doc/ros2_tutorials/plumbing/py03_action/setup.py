from setuptools import setup

package_name = 'py03_action'

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
            'demo01_action_server_py = py03_action.demo01_action_server_py:main',
            'demo02_action_client_py = py03_action.demo02_action_client_py:main'
        ],
    },
)
