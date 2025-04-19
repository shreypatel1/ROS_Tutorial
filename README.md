# Robotics Coding Tutorial
In this repo, you will learn the basics of ROS2 and robotics fundamentals. The goal of this tutorial is start with the basics and slowly progress to creating your own full autonomy stack for the stinger tugs. Throughout this tutorial, you will fill out code blocks, answer questions, and write code. After completing each section, there will be a provided autograder that will verify your work. Only by passing all the required autograder tests should you move onto the next section. If you are having a hard time passing some tests, please reach out! We are more than happy to help.

## Prerequisites
- **Some base knowledge of coding/python** If you are not yet familiar with python, you can still attempt to go through this onboarding process but it is highly recommended that you learn python first. Some great tutorials are:
  - https://www.w3schools.com/python/
  - https://docs.python.org/3/tutorial/index.html
- Access to your MRG Cluster Virtual Machine
- Basics of Navigating Linux

## Topic 0: Building + Testing
Each section will give a rough description of the task and there will be an associated file in ```ROS_Tutorial/student_code/student_code/question_{section #}_{subsection #}.py```. So for example, if you were working on ```section 1.2```, you would be accessing the file ```ROS_Tutorial/student_code/student_code/question_1_2.py```

Each section will also have an associated autograder. At any point you want to test your code, you will need to run these commands in your workspace directory. For this tutorial, the workspace directory refers to the folder location of ```ROS_Tutorial/```.
```
colcon build
source install/setup.bash
ros2 run autograder test_topic_{topic_number}_{topic_subsection}
```
For example, to run the tests for ```section 1.2```, you should run `ros2 run onboarding test_topic_1_2`

**Note:** Building your code is super important for ensuring that your latest changes are reflected when you execute your code.

## Topic 1: ROS2 Basics
### 1.1 Understanding Nodes and Topics
The goal of this section is to familiarize yourself with the concept of nodes and topics. We will be using ROS2 CLI (Command Line Interface) throughout this section.
