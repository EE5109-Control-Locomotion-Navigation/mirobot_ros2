from setuptools import setup

package_name = 'my_ros2_package'  # Replace with your package name

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # No package directories, only scripts
    py_modules=[
        'scripts.mirobot_gcode_writer',  # Module in the scripts directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # Replace with your name
    maintainer_email='your_email@example.com',  # Replace with your email
    description='A ROS 2 package that includes a script for mirobot gcode writing',
    license='Your License',  # Specify your license
    entry_points={
        'console_scripts': [
            # Create a console command 'mirobot_gcode_writer' that calls the main() in mirobot_gcode_writer.py
            'mirobot_gcode_writer = scripts.mirobot_gcode_writer:main',
        ],
    },
)