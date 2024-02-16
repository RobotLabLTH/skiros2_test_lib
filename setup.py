from setuptools import setup

package_name = 'skiros2_test_lib'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/test.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
)
