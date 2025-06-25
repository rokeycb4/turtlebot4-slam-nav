from setuptools import find_packages, setup

package_name = 'rokey_pjt'

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
    maintainer='kiwi',
    maintainer_email='mingureion@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_checker = rokey_pjt.depth_checker:main',
            'depth_checker_mouse_click = rokey_pjt.depth_checker_mouse_click:main',

            'tf_point_transform = rokey_pjt.tf_point_transform:main',


        ],
    },
)
