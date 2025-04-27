from setuptools import find_packages, setup

package_name = 'ela2_py_arms'

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
    maintainer='abdel',
    maintainer_email='abdel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = ela2_py_arms.simple_publisher:main',
            'simple_subs = ela2_py_arms.simple_subs:main',
            'simple_parameter = ela2_py_arms.simple_parameter:main',
            
        ],
    },
)
