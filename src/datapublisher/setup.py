from setuptools import setup

package_name = 'datapublisher'

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
    entry_points={
        'console_scripts': [
            'csv_sender_node = datapublisher.csv_sender:main',
            'csv_processor_node = datapublisher.csv_processor:main',
        ],
    },
)
