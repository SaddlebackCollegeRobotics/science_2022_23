from setuptools import setup

package_name = 'pmt_oscilloscope'

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
    maintainer='jasper',
    maintainer_email='49565505+wluxie@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pico_pub = pmt_oscilloscope.pico_pub:main',
            'pico_sub = pmt_oscilloscope.pico_sub:main',
        ],
    },
)
