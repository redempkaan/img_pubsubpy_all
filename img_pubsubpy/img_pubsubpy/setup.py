from setuptools import find_packages, setup

package_name = 'img_pubsubpy'

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
    maintainer='kredemp',
    maintainer_email='kaanmenevse@hotmail.com',
    description='Image Publisher',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = img_pubsubpy.publisher_member_function:main',
            'listener = img_pubsubpy.subscriber_member_function:main',
            'listener2 = img_pubsubpy.subscriber_member_function2:main',
        ],
    },
)
