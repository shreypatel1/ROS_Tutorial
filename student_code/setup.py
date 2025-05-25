from setuptools import find_packages, setup

package_name = 'student_code'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'question_1_2 = student_code.topic_1.question_1_2:main',
            'question_1_3 = student_code.topic_1.question_1_3:main',
            'question_4_3 = student_code.topic_4.question_4_3:main',
        ],
    },
)
