# Topics: Sending and Receiving Messages between Nodes
**Table of Contents**
- [Topics: Sending and Receiving Messages between Nodes](#topics-sending-and-receiving-messages-between-nodes)
  - [What is a ROS 2 Topic?](#what-is-a-ros-2-topic)
  - [Writing a Publisher Node](#writing-a-publisher-node)
    - [Writing a Python Publisher](#writing-a-python-publisher)
    - [Writing a C++ Publisher](#writing-a-c-publisher)
  - [Writing a Subscriber Node](#writing-a-subscriber-node)
    - [Writing a Python Subscriber](#writing-a-python-subscriber)
    - [Writing a C++ Subscriber](#writing-a-c-subscriber)
  - [Additional tools to handle topics](#additional-tools-to-handle-topics)
    - [The ros2 topic command line tool](#the-ros2-topic-command-line-tool)
  - [\[IMPORTANT\] Creating a custom interface for a topic](#important-creating-a-custom-interface-for-a-topic)
    - [Using an existing interface in your node](#using-an-existing-interface-in-your-node)
    - [Creating a new topic interface](#creating-a-new-topic-interface)
    - [Using the custom interface in your nodes](#using-the-custom-interface-in-your-nodes)
## What is a ROS 2 Topic?
This is what defines a topic: a name and an interface. \
Here are some important points about how topics work:
* A topic is defined by a **name** and an **interface (message type)**.
* A topic name must start with a letter and can be followed by letters, numbers, underscores, and slashes.
* Any publisher or subscriber to a topic must use the same interface (message type).
* Publishers and subscribers are anonymous; they are not aware of each other; they just know they are publishing or subscribing to a topic.
* A node can contain multiple publishers and subscribers, even for the same topic.

## Writing a Publisher Node
### Writing a Python Publisher
Navigate inside the `my_py_pkg` package, create a Python file, and make it executable:
```bash
$ cd ~/ros2_ws/src/my_py_pkg/my_py_pkg
$ touch number_publisher.py
$ chmod +x number_publisher.py
```
Open this file, use the node OOP template and modify the required fields to give names that make sense:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
For the interface, you have two choices: use an existing interface or create a custom one. Let's use `example_interfaces/msg/Int64`. To get more details about this interface:
```bash
$ ros2 interface show example_interfaces/msg/Int64
# Some commets
int64 data
```
Let's create the publisher. First, import the interface, and then create the publisher in the constructor.
```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.number_publisher_ = self.create_publisher(Int64, 'number', 10)
```
To import the interface, we must specify the name of the package (`example_interfaces`), then the folder name for topic messages (`msg`), and finally the class for the interface (`Int64`).

To create the publisher, we use the `create_publisher()` method from the `Node` class. Inheriting from this class gives us access to all ROS 2 functionalities. This method requires three arguments:
* **Topic interface**: We'll use `Int64` from the `example_interfaces` package.
* **Topic name**: As defined previously, this is `number`.
* **Queue size**: If the messages are published too fast and subscribers can't keep up, messages will be buffered so that they're not lost. This is particularly important if you send large messages (images) at a high frequency over a lossy network.
```python
def __init__(self):
    super().__init__('number_publisher')
    self.number_ = 2
    self.number_publisher_ = self.create_publisher(Int64, 'number', 10)
    self.number_timer_ = self.create_timer(1.0, self.publish_number)
    self.get_logger().info('Number publisher node has been started.')

def publish_number(self):
    msg = Int64()
    msg.data = self.number_
    self.number_publisher_.publish(msg)
```
We create an object from the `Int64` class. This object contains a `data` field. We publish the message using the `publish()` method from the publisher.

Since we're using a new dependency (`example_interfaces`), we also need to add one line to the `package.xml` file inside the `my_py_pkg` package:
```xml
<depend>rclpy</depend>
<depend>example_interfaces</depend>
```
To register the node, open the `setup.py` file inside the `my_py_pkg` package and add the following lines to the `entry_points` section:
```python
entry_points={
    'console_scripts': [
        'test_node = my_py_pkg.my_first_node:main',
        'number_publisher = my_py_pkg.number_publisher:main'
    ],
},
```
Make sure you add a comma between each line.

Build and run the node:
```bash
$ cd ~/ros2_ws/
$ colcon build --packages-select my_py_pkg
$ source install/setup.bash
$ ros2 run my_py_pkg number_publisher
[INFO] [number_publisher]: Number publisher node has been started.
$ ros2 topic list
/number
/param_events
/rosout
$ ros2 topic echo /number
data: 2
data: 2
...
```
As you can see, there is a leading slash (`/`) added before the topic name. We only wrote `number` in the code, not `/number`. This is because ROS 2 names (nodes, topics, and so on) are organized inside namespaces. Later, we will see that you can add a namespace to put all your nodes and topics inside it. By default, all nodes and topics are inside the root namespace (`/`), which we call the global namespace.

### Writing a C++ Publisher
```bash
$ cd ~/ros2_ws/src/my_cpp_pkg/src
$ touch number_publisher.cpp
```
To include an interface for a topic, use `<package_name>/msg/<interface_name>.hpp>`.
```c++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
```
Then add the following in the constructor:
```c++
number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
```
In C++, we also use the `create_publisher()` method from the `Node` class, but the syntax is a bit different. We use the template syntax to specify the interface type. The rest of the arguments are the same as in Python.

The publisher is also declared as a private attribute of the class:
```c++
rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
```
As you can see, we use the `rclcpp::Publisher` class, and like many things in ROS 2, we use a shared pointer. For several common classes, ROS 2 provides `::SharedPtr`, which is equivalent to writing `std::shared_ptr<the publisher class>`.
```c++
void publish_number()
{
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
}
```
As we did for Python, open the `package.xml` file inside the `my_cpp_pkg` package and add the following line:
```xml
<depend>rclcpp</depend>
<depend>example_interfaces</depend>
```
Open the `CMakeLists.txt` file inside the `my_cpp_pkg` package and add the following lines to the `find_package()` section.

Then we create the executable with `add_executable` and link the dependencies with `ament_target_dependencies`.

Finally, there is no need to re-create the `install()` block if it already exists; just add the new executable to the existing block.
```CMake
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)
install(TARGETS
  test_node
  number_publisher
  DESTINATION lib/${PROJECT_NAME}
)
```
Build and run the node:
```bash
$ cd ~/ros2_ws/
$ colcon build --packages-select my_cpp_pkg
$ source install/setup.bash
$ ros2 run my_cpp_pkg number_publisher
[INFO] [number_publisher]: Number publisher node has been started.
$ ros2 topic echo /number
data: 2
data: 2
...
```

## Writing a Subscriber Node
### Writing a Python Subscriber
```bash
$ cd ~/ros2_ws/src/my_py_pkg/my_py_pkg
$ touch number_subscriber.py
$ chmod +x number_subscriber.py
```
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.number_subscription_ = self.create_subscription(
            Int64,
            'number',
            self.callback_number,
            10)
        self.get_logger().info('Number subscriber node has been started.')
```
We create a subscriber in the node's constructor. Here, we use the `create_subscription()` method from the `Node` class. This method requires four arguments:
* **Topic interface**: `Int64` from the `example_interfaces` package. Same as the publisher.
* **Topic name**: `number`. Same as the publisher.
* **Callback function**: In ROS 2, callbacks are fundamental. We use a callback method for the subscriber. When the node is spinning, it will stay alive and all registered callbacks will be ready to be called.
* **Queue size**: Same as the publisher.

For the callback methods, we usually use `callback_` prefix to make it clear that this method shouldn't be called directly in the code.

```python
def callback_number(self, msg: Int64):
    self.counter_ += msg.data
    self.get_logger().info("Counter: " + str(self.counter_))
```
In a subscriber callback method, we receive the message directly as a parameter of the function. Since we know that `Int64` has a `data` field, we can access it using `msg.data`.

As a best practice, I have specified the `Int64` type for the `msg` parameter. This is not mandatory in Python, but it adds an extra level of safety.

Add a new executable in the `setup.py` file inside the `my_py_pkg` package:
```python
entry_points={
    'console_scripts': [
        'test_node = my_py_pkg.my_first_node:main',
        'number_publisher = my_py_pkg.number_publisher:main',
        'number_subscriber = my_py_pkg.number_subscriber:main'
    ],
},
```
Build and run the node:
```bash
$ cd ~/ros2_ws/
$ colcon build --packages-select my_py_pkg
$ source install/setup.bash
$ ros2 run my_py_pkg number_publisher
[INFO] [number_publisher]: Number publisher node has been started.
$ ros2 run my_py_pkg number_subscriber
[INFO] [number_counter]: Number counter node has been started.
[INFO] [number_counter]: Counter: 2
[INFO] [number_counter]: Counter: 4
[INFO] [number_counter]: Counter: 6
...
```
### Writing a C++ Subscriber
```bash
$ cd ~/ros2_ws/src/my_cpp_pkg/src
$ touch number_subscriber.cpp
```
```c++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
                "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, _1));

        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
    }

    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
In C++, we find the same components as Python: topic interface, topic name, queue size, and callback function. The order of the arguments is a bit different. For `_1` to work, don't forget to include `using namespace std::placeholders;` at the beginning of the file.

The subscriber object is declared as a private attribute of the class, using the `rclcpp::Subscription` class with a shared pointer.

We then have the callback method, which receives a shared pointer to the message. We access the `data` field using the arrow operator (`->`).

Add a new executable in the `CMakeLists.txt` file inside the `my_cpp_pkg` package:
```CMake
add_executable(number_subscriber src/number_subscriber.cpp)
ament_target_dependencies(number_subscriber rclcpp example_interfaces)

install(TARGETS
  test_node
  number_publisher
  number_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
```
Run the C++ node and Python node together:
```bash
$ cd ~/ros2_ws/
$ colcon build --packages-select my_cpp_pkg
$ source install/setup.bash
$ ros2 run my_py_pkg number_publisher
[INFO] [number_publisher]: Number publisher node has been started.
$ ros2 run my_cpp_pkg number_subscriber
[INFO] [number_counter]: Number Counter has been started.
[INFO] [number_counter]: Counter: 2
[INFO] [number_counter]: Counter: 4
[INFO] [number_counter]: Counter: 6
...
```
ROS 2 communications happen at a lower level, using **Data Distribution Service (DDS)**. This is the middleware part and is responsible for sending and receiving messages between nodes. When you write a Python or C++ node, you are using the same DDS functionality, with an API implemented in either `rclpy` or `rclcpp`. 

## Additional tools to handle topics
### The ros2 topic command line tool
List all topics:
```bash
$ ros2 topic list
/number
/param_events
/rosout
$ ros2 topic info /number
Type: example_interfaces/msg/Int64
Publisher count: 1
Subscriber count: 1
```
Get more details about the topic's interface:
```bash
$ ros2 interface show example_interfaces/msg/Int64
# Some commets
int64 data
```
Subscribe to a topic and print messages to the console:
```bash
$ ros2 topic echo /number
data: 2
data: 2
data: 2
...
```
Publish messages to a topic from the command line, `-r` specifies the rate in Hz:
```bash
$ ros2 topic pub -r 2.0 /number example_interfaces/msg/Int64 "{data: 7}"
publisher: beginning loop
publishing #1: example_interfaces/msg/Int64(data=7)
publishing #2: example_interfaces/msg/Int64(data=7)
publishing #3: example_interfaces/msg/Int64(data=7)
...
```
Both `ros2 topic echo` and `ros2 topic pub` can save you time when collaborating with other people on a project.

Change a topic name at runtime using remapping, adding `-rr` followed by `<old_topic_name>:=<new_topic_name>`:
```bash
$ ros2 run my_py_pkg number_subscriber --ros-args -r number:=my_number
```
Changing topic names at runtime will be quite useful, especially when you want to run several existing nodes that you cannot modify. Even if you cannot rewrite the code, you can modify the names at runtime.

Record messages from a topic into a bag file:
```bash
$ mkdir ~/bags
$ cd ~/bags
$ ros2 bag record /number -o bag1
...
[INFO] [rosbag2_recorder]: Subscribed to topic '/number'
[INFO] [rosbag2_recorder]: Recording...
[INFO] [rosbag2_recorder]: All request topic subscriptions are subscribed. Stopping discovery...
```
Stop the recording using `Ctrl+C`. In the directory, you will find `.mcap` files that contain the recorded messages and a YAML file with more information. If you open this YAML file, you will see the recorded duration, number of messages, and topics that were recorded.
You can then play back the bag file:
```bash
$ ros2 bag play ~/bags/bag1
```

## [IMPORTANT] Creating a custom interface for a topic
You can find the most common interfaces in the `ros2 interface` repository: https://github.com/ros2/common_interfaces.
### Using an existing interface in your node
Let's say you want to create a driver node for camera and publish the images to a topic. You can use the existing `sensor_msgs/msg/Image` interface.
```bash
$ sudo apt install ros-<ros2-distro>-sensor-msgs
```
Then, you can find the details of this interface:
```bash
$ ros2 interface show sensor_msgs/msg/Image
# This message contains an image
Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```
To use this interface in your node, just import it like any other interface.
1. In the `package.xml` file of the package where you write your node, add the dependency to the interface package.
2. In your node, import the message and use it in your publisher or subscriber.
3. For C++ only, make sure to add the dependency in the `CMakeLists.txt` file.

### Creating a new topic interface
Before we create any topic interface(message), we need to create a new package and set it up for building interfaces. As a best practice, in your application, you will have one package dedicated to custom interfaces. This means that you create interfaces only in this package, and you keep this package only for interfaces - no nodes or other things, just interfaces. \
A common practice when naming this interface package is to start with the name of your application or robot and add the `_interfaces` suffix. For example, if your robot is called `my_robot`, you can name the interface package `my_robot_interfaces`.
```bash
$ cd ~/ros2_ws/src
$ ros2 pkg create my_robot_interfaces
```
At this point, your workspace should contain three packages: `my_py_pkg`, `my_cpp_pkg`, and `my_robot_interfaces`. \
We need to set this new package up and modify a few things so it can build messages. Go into the package, remove `src` and `include` directories, and create a new `msg` folder:
```bash
$ cd ~/ros2_ws/src/my_robot_interfaces
$ rm -r src include
$ mkdir msg
```
Open the `package.xml` file, and add the following lines after the `<buildtool_depend>ament_cmake</buildtool_depend>` line:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Open the `CMakeLists.txt` file, and make sure it contains the following lines after the `find_package(ament_cmake REQUIRED)` line (you can also remove the `if(BUILD_TESTING)` block if it exists):
```CMake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  # we will add the name of our custom messages here later
)
ament_export_dependencies(rosidl_default_runtime)
```
We have some rules for the new interface:
* Use UpperCamelCase for the name. For example, HardwareStatus.
* Don't write `Msg` or `Interface` in the name as this would add unnecessary redundancy.
* **Use .msg as the file extension.**
```bash
$ cd ~/ros2_ws/src/my_robot_interfaces/msg/
$ touch HardwareStatus.msg
```
Inside this file, we can add the definition of the message:
* **Built-in types**: `bool`, `byte`, `char`, `float32`, `float64`, `int8`, `int16`, `int32`, `int64`, `uint8`, `uint16`, `uint32`, `uint64`, `string`, as well as arrays of these types. You can find the complete list [here](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html#field-types).
* **Other existing messages**: Use the package name, followed by the message name. For example, `geometry_msgs/Twist`.
```text
int64 version
float64 temperature
bool are_motors_ready
string debug_message
```
To build the message, you simple have to add one line to the `rosidl_generate_interfaces()` block in the `CMakeLists.txt` file:
```CMake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```
Now, go back to the root of your workspace and build the new interface package:
```bash
$ cd ~/ros2_ws/
$ colcon build --packages-select my_robot_interfaces
Starting >>> my_robot_interfaces
Finished <<< my_robot_interfaces [X.XXs]
Summary: 1 package finished [X.XXs]
```
You can see the interface from the command line:
```bash
$ source install/setup.bash
$ ros2 interface show my_robot_interfaces/msg/HardwareStatus
int64 version
float64 temperature
bool are_motors_ready
string debug_message
```
### Using the custom interface in your nodes
Now, you can use this new interface in your publisher and subscriber nodes by importing it from the `my_robot_interfaces` package. Don't forget to add the dependency in the `package.xml` files of the packages where you write your nodes.
```xml
<depend>my_robot_interfaces</depend>
```
For Python:
1. Import the interface in Python

```python
from my_robot_interfaces.msg import HardwareStatus
```
2. Create the publisher and specify the interface
```Python
self.hardware_status_publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
```
3. Create the message, fill in the fields, and publish it

```python
msg = HardwareStatus()
msg.version = 1
msg.temperature = 36.5
msg.are_motors_ready = True
msg.debug_message = "All systems are operational."
self.hardware_status_publisher_.publish(msg)
```
For C++:
1. Import the interface in C++
```c++
#include "my_robot_interfaces/msg/hardware_status.hpp"
```
2. Create the publisher and specify the interface
```c++
hardware_status_publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
```
3. Create the message, fill in the fields, and publish it

```cpp
auto msg = my_robot_interfaces::msg::HardwareStatus();
msg.version = 1;
msg.temperature = 36.5;
msg.are_motors_ready = true;
msg.debug_message = "All systems are operational.";
hardware_status_publisher_->publish(msg);
```
As you've seen, first check if there is an existing interface that fits your needs. If there is, don't reinvent the wheel. If not, don't hesitate to create your own custom interface. To do that, you **must** create a dedicated package for your interfaces.
