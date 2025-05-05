# Robotics Coding Tutorial
In this repo, you will learn the basics of ROS2 and robotics fundamentals. The goal of this tutorial is start with the basics and slowly progress to creating your own full autonomy stack for the stinger tugs. Throughout this tutorial, you will fill out code blocks, answer questions, and write code. After completing each section, there will be a provided autograder that will verify your work. Only by passing all the required autograder tests should you move onto the next section. If you are having a hard time passing some tests, please reach out! We are more than happy to help.

## Prerequisites
- **Some base knowledge of coding/python** If you are not yet familiar with python, you can still attempt to go through this tutorial but it is highly recommended that you learn python first. Some great tutorials are:
  - https://www.w3schools.com/python/
  - https://docs.python.org/3/tutorial/index.html
- Access to your MRG Cluster Virtual Machine
- Basics of Navigating Linux

## Topic 0: Building + Testing
Each topic will give a rough description of the task and there will be an associated file in ```ROS_Tutorial/student_code/student_code/topic_{topic #}/question_{topic #}_{section #}.py```. So for example, if you were working on ```Topic 1.2```, you would be accessing the file ```ROS_Tutorial/student_code/student_code/topic_1/question_1_2.py```

Each section will also have an associated autograder. At any point you want to test your code, you will need to run these commands in your workspace directory. For this tutorial, the workspace directory refers to the folder location of ```ROS_Tutorial/```.
```
colcon build
source install/setup.bash
ros2 run autograder test_topic_{topic_number}_{topic_subsection}
```
For example, to run the tests for ```topic 1.2```, you should run `ros2 run autograder test_topic_1_2`. 

**Note:** Make sure nothing is running in the background when running the autograder tests. Any background processes may mess up the autograder.

**Note:** Building your code is super important for ensuring that your latest changes are reflected when you execute your code.

## Topic 1: ROS2 Basics
### 1.1 Understanding Nodes and Topics
The goal of this section is to familiarize yourself with the concept of nodes and topics. We will be using ROS2 CLI (Command Line Interface) throughout this section.

#### 1.1.a Nodes
> **Nodes:** A node in ROS2 represents a process that performs computation, such as sensing, control, planning, or actuation, and typically communicates with other nodes using the ROS2 communication framework. 

First in your terminal, run `ros2 node list`. This will list all running nodes. You should currently have 0 running nodes. Now, in the terminal, run `ros2 run helpers node_q_1_1`. After you run the node, how many nodes are now running? Change the value of `num_nodes` (in ```question_1_1.py```) to the new number of nodes. 

#### 1.1.b Node Names
Change the value of `first_node_name` to the name of the first node. Make sure you include the starting ```/```.

**Note:** When we ran the node, we used the command `ros2 run helpers node_q_1_1`. As you can see, the name `node_q_1_1` will not always 
match the name of the node. Here, `node_q_1_1` refers to the executable name, which is defined in `setup.py`.

#### 1.1.c Topics

> **Topics:** A topic is a communication channel that allows for messages to be sent between nodes.
**Messages**: A message is an object that contains information such as an image, a velocity, etc.

Stop the node. This can be done by going into the terminal where you ran the node and press `CTRL + c`. Once you have killed the node,
run `ros2 topic list`. You should see two topics, namely `/parameter_events` and `/rosout`. These are system-generated topics and you do not have to worry about these topics for now. Again, run `ros2 run helpers node_q_1_1`. Change the value of `num_topics` to the new number of topics. 

#### 1.1.d Topic Info
Sometimes you may want to find information about a topic. You can use `ros2 topic info TOPIC_NAME` to do so. First, run `ros2 topic info /tutorial/OdometryPub`. Note the fields `Type`, `Publisher count`, and `Subscription count`. One of the other topics is called `/tutorial/MysteryPub`. We want to find what type of message this topic publishes. Change the value of `topic_message_type` to the correct message type of the topic `/tutorial/MysteryPub`. Write your answer as a string. For example, the answer for ```/tutorial/OdometryPub``` would be ```"nav_msgs/msgs/Odometry"```. 

#### 1.1.e Topic Echo
When debugging, it sometimes important to check out what messages are being published to a topic. A useful command for this is `ros2 topic echo TOPIC_NAME`. Change the value of `string_message` to the string that is being published to the topic `/tutorial/StringPub`. 

**Note:** do not include ```"data: "``` in your answer.

### 1.2 Coding Subscriber and Publishers
The goal of this section is to understand what a publisher and subscriber is within ROS2 and how to create them. Please look at file `question1_2.py`. You may run ```ros2 run helpers node_q_1_2.py``` to debug your answer.

#### 1.2.a Creating a subscriber to a topic

> **Subscribers/Subscription:** A subscriber is a component of a node that allows the node to receive information and data from a topic.

For this question, fill in the blanks to create a subscriber.
This subscriber should
- Take in a message type `String`
- Subscribe to the topic `/tutorial/basic_topic`
- Have its callback be `self.topic_callback`
- Qos profile of 10

**HINT:** There should be four parameters that you fill

#### 1.2.b Creating a publisher
> **Publishers:** A publisher is a component of a node that allows the node to send information and data to a topic.

For this question, fill in the blanks to create a publisher.
This publisher should
- Publish a message type `String`
- Publish to a topic `/tutorial/new_topic`
- Qos profile of 10

**HINT:** There should be three parameters that you fill

#### 1.2.c Message Data
Please see what the String message type consists of: https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html.
For this question, access the string field from the variable `msg` and store it into the variable `self.topic_string_message`

#### 1.2.d Publishing Data
For this question, take the value from `topic_string_message` and append the string ` ROS` (don't forget the space). The new string should look like `Hello World! ROS`. Then use the variable `new_message` to publish this new string using the publisher from **Q1.1.b**.

### 1.3 Counter Node
This question is designed to test your knowledge on this topic. Take a look at `question_1_3.py`. The goal of this node is to publish to a topic called `/tutorial/counter25` with numbers of type `Int32` starting from 0 incrementing to 25 inclusive. A rough outline has been provided for you. Fill in the blanks to complete this question.

**HINT:** It may be useful to debug your node using `ros2 run student_code question_1_3` and `ros2 topic echo /tutorial/counter25`.

## Topic 2: Coordinate Conventions

## Topic 3: TF Transformations

## Topic 4: Simulation + Useful Tools

## Topic 5: Localization

## Topic 6: Perception 

## Topic 7: Control

## Topic 8: Autonomy

