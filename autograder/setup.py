from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autograder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*[pxy][yma]*'))),
         (os.path.join('share', package_name, 'resources'),
         glob(os.path.join('resources', '*png'))),
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
            'test_topic_3_2 = autograder.topic_3.question_3_2_grader:main',
            'test_topic_4_3 = autograder.topic_4.question_4_3_grader:main',
            'test_topic_4_4 = autograder.topic_4.question_4_4_grader:main',
            'test_topic_5_1 = autograder.topic_5.question_5_1_grader:main',
            'test_topic_6_1 = autograder.topic_6.question_6_1_grader:main',
        ],
    },
)
