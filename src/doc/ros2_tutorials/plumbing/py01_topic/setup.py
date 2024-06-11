from setuptools import setup

package_name = 'py01_topic'

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
            'demo01_publihser_str_py = py01_topic.demo01_publisher_str_py:main',
            'demo02_subscriber_str_py = py01_topic.demo02_subscriber_str_py:main',
            'demo03_publihser_custom_py = py01_topic.demo03_publisher_custom_py:main',
            'demo04_subscriber_custom_py = py01_topic.demo04_subscriber_custom_py:main',
        ],
    },
)
