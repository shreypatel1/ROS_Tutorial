from setuptools import find_packages, setup

package_name = 'autograder'

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
            'test_topic_1_1 = autograder.topic_1.question_1_1_grader:main',
            'test_topic_1_2 = autograder.topic_1.question_1_2_grader:main',
            'test_topic_1_3 = autograder.topic_1.question_1_3_grader:main',
            'test_topic_1_4 = autograder.topic_1.question_1_4_grader:main',
        ],
    },
)
