# Service: Client/Server Interaction between Nodes

**Table of Contents**
- [Service: Client/Server Interaction between Nodes](#service-clientserver-interaction-between-nodes)
  - [What is a ROS 2 Service?](#what-is-a-ros-2-service)
  - [Creating a custom service interface](#creating-a-custom-service-interface)
    - [Finding an existing service interface](#finding-an-existing-service-interface)
    - [Creating a new service interface](#creating-a-new-service-interface)
  - [Writing a service server](#writing-a-service-server)
    - [Writing a Python service server](#writing-a-python-service-server)
    - [Writing a C++ service server](#writing-a-c-service-server)
  - [Writing a service client](#writing-a-service-client)
    - [Writing a Python service client](#writing-a-python-service-client)
    - [Writing a C++ service client](#writing-a-c-service-client)


## What is a ROS 2 Service?
As you can see, a service , just link for a topics, has a name and an interface. This interface is not just one message, it's a pair of messages: a request message and a response. Both the client and server nodes must use the same name and interface to successfully communicate with each other. \
When should you use topics versus services? You should use topics to publish unidirectional data streams and services when you want to have a client/server type of communication. \
Here are some important points about how services work:
* A service is defined by a name and an interface.
* The name of service follows the same rules as topics. It must start with a letter and can be followed by letters, numbers, underscores, tildes, and slashes.
* The interface contains two things: **a request message and a response message**. Both the client and server nodes must use the same interface to be able to communicate with each other.
* A service server can only exist once but can have multiple clients.
* Service clients are not aware of each other and not aware of the server node. To reach the server, they just know that they must use the service name and provide the correct interface.
* One node can contain multiple service clients and servers, each with different service name.
* Services are synchronous. When a client sends a request to the server, it waits (blocks) until it receives a response from the server.
* Services are not designed for high-frequency communication. If you need to send data at a high frequency, you should use topics instead.
* Services are often used for remote procedure calls (RPCs) in ROS 2, where a client node requests a specific action or computation from a server node and waits for the result.

## Creating a custom service interface
At this point, you know that you must avoid using the  `example_interface` package for real applications, but for a fist test, that wasn't a problem. \
### Finding an existing service interface
```bash
$ ros2 interface list | grep example_interfaces/srv
example_interfaces/srv/AddTwoInts
example_interfaces/srv/SetBool
eample_interfaces/srv/Trigger

$ ros2 interface list | grep std_srvs/srv
std_srvs/srv/Empty
std_srvs/srv/SetBool
std_srvs/srv/Trigger
```
You could have a look at other interfaces in the common interfaces GitHub repository (https://github.com/ros2/common_interfaces) but you won't find exactly what we are looking for.

### Creating a new service interface
First, navigate inside the `my_robot_interfaces` package(where you already have a `msg` folder) and create a new folder named `srv`:
```bash
$ cd ~/ros2_ws/src/my_robot_interfaces
$ mkdir srv
```
Here are the rules to follow regarding the filename:
* Use UpperCamelCase, For example: `ActiveMotor`
* Do not write `Srv` or `Interface` in the name as this would add unnecessary redundancy.
* **Use `.srv` as the file extension.**
* As a best practice, use a verb in the interface name. For example, `TriggerSomething`, `ActivateMotor`, or `ComputeDistance`. Services are about doing an action or computation, so using a verb, you make it very clear what the service is doing.
Since we want to reset the counter, we can name the service `ResetCounter.srv`:
```bash
$ cd ~/ros2_ws/src/my_robot_interfaces/srv
$ touch ResetCounter.srv
```
Open this file and write the definition of the service interface. One very important thing to do here is add three dashes `---` to separate the request and response parts of the service interface. \
For the request and response, you can use the following:
* Built-in types: `int32`, `float64`, `string`, `bool`, etc.
* Existing message interfaces: You can use any message type defined in other packages, such as `std_msgs/msg/String`, `geometry_msgs/msg/Point`, etc.
Let's write our service interface. As it's not too complex, we can use simply built-in types:
```text
int64 reset_value
---
bool success
string message
```
**All the fields inside the definition must follow the snake_case naming convention(use lowercase letters and underscores to separate words).** \
Make sure you always have the three dashes `---`  in all your service definitions, even if the request or response is empty. \
Go back to the `CMakeLists.txt` file of the `my_robot_interfaces` package. Since the package has already been configured, we just need to add one line. Add the relative path to the interface on the new line inside the `rosidl_generate_interfaces()` function. Do not use any commas between the interface files:
```CMake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  "srv/ResetCounter.srv"
)
```
After this, save all files and build `my_robot_interfaces` package:
```bash
$ cd ~/ros2_ws
$ colcon build --packages-select my_robot_interfaces
```
Once built, source the environment. You should be able to find your new service interface:
```bash
$ source install/setup.bash
$ ros2 interface show my_robot_interfaces/srv/ResetCounter
int64 reset_value
---
bool success
string message
```

## Writing a service server
To write a service server, you will need to import the interface and then create a new service in the node's constructor.
### Writing a Python service server
First, we need to add the dependency to this interface package inside the package where we write the node with the service. Open the `package.xml` file from the `my_robot_services` package and add new dependency:
```xml
<depend>rclpy</depend>
<depend>example_interfaces</depend>
<depend>my_robot_interfaces</depend>
```
Now, import the dependency into your code(`number_counter.py`):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from my_robot_interfaces.srv import ResetCounter
```
To import a service interface, we must specify the package name(`my_robot_interfaces`), followed by the name for service(`srv`), and finally the class for the interface(`ResetCounter`). \
Add a service server to the node's constructor:
```python
class NumberCounter(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter_ = 0
        self.number_subscription = self.create_subscription(Int64, 'number', self.callback_number, 10)
        self.reset_counter_service = self.create_service(ResetCounter, 'reset_counter', self.callback_reset_counter)
        self.get_logger().info('Number Counter has been started.')
```
To create the service server, we use the `create_service()` method from the `Node` class. Once again, you can see that by inheriting from the `Node` class, we get access to all ROS 2 functionalities easily. This method requires three arguments:
* **Servcie Interface**: The first argument is the service interface class(`ResetCounter`) that we imported at the beginning of the file.
* **Service Name**: The second argument is the name of the service(`reset_counter`).
* **Callback Function**: The third argument is the callback function(`callback_reset_counter`) that will be called whenever a client sends a request to this service. While the node is spinning, the server will be in "waiting" mode. Upon reception of a request, the service callback will be triggered. and the request will be passed to this callback.

Now, let's implement the callback function for the service server:
```python
def callback_reset_counter(self, request: ResetCounter.Request, response: ResetCounter.Response):
    self.counter_ = request.reset_value
    self.get_logger().info("Resetting counter to" + {request.counter_})
    response.success = True
    response.message = "Success"
    return response
```
In a service callback, we receive two things: an object for the request and an object for the response. The request object contains all the data sent by the client. The response object is empty, and we will need to fill it, as well as return it. \
To name the callback, I usually write `callback_` followed by the service name in snake_case. \
In the method's arguments, I have also specified the type for the two arguments: `ResetCounter.Request` and `ResetCounter.Response`. We get two classes from the service interface, while only one class from the topic interface. 
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from my_robot_interfaces.srv import ResetCounter


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.number_subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.reset_counter_service_ = self.create_service(ResetCounter, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self, msg: Int64):
        self.counter_ += msg.data
        self.get_logger().info("Counter:  " + str(self.counter_))

    def callback_reset_counter(self, request: ResetCounter.Request, response: ResetCounter.Response):
        if request.reset_value < 0:
            response.success = False
            response.message = "Cannot reset counter to a negative value"
        elif request.reset_value > self.counter_:
            response.success = False
            response.message = "Reset value must be lower than current counter value"
        else:
            self.counter_ = request.reset_value
            self.get_logger().info("Reset counter to " + str(self.counter_))
            response.success = True
            response.message = "Success"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
```
```bash
$ ros2 run my_py_pkg number_counter
[INFO] [number_counter]: Number Counter has been started.
```

### Writing a C++ service server
Since we will have a dependency to the `my_robot_interfaces` package, we need to add it to the `CMakeLists.txt` and `package.xml` files of the `my_cpp_pkg` package.
```CMake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces my_robot_interfaces)
```
```xml
<depend>rclcpp</depend>
<depend>example_interfaces</depend>
<depend>my_robot_interfaces</depend>
```
Then, open the `number_counter.cpp` file and import the service interface:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "my_robot_interfaces/srv/reset_counter.hpp"

using ResetCounter = my_robot_interfaces::srv::ResetCounter;
```
To import a service interface in C++, you must use `#include "<package_name>/srv/<service_name>.hpp"`. After this, I added an extra line with the `using` keyword so that we can just write `ResetCounter` instead of the full name `my_robot_interfaces::srv::ResetCounter` every time we need to use this interface. \
Now, let's create the service server inside the node's constructor:
```cpp
rclcpp::Service<ResetCounter>::SharedPtr reset_counter_service_;
```
As you can see, we use the `rclcpp::Service` class, and then, as always, we make it a shared pointer with `::SharedPtr`. Inside the angle brackets, we specify the service interface class(`ResetCounter`). \
Now, we can initialize the service server inside the constructor:
```cpp
reset_counter_service_ = this->create_service<ResetCounter>(
    "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
```
To create the service server, we use the `create_service<>()` method from the `rclcpp::Node` class. For `_1` and `_2`, make sure to add `using namespace std::placeholders;` at the beginning of the file after the `#include` statements. \
Followings are all the code for the C++ service server:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "my_robot_interfaces/srv/reset_counter.hpp"

using namespace std::placeholders;
using ResetCounter = my_robot_interfaces::srv::ResetCounter;
using Int64 = example_interfaces::msg::Int64;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
                "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, _1));
        reset_counter_service_ = this->create_service<ResetCounter>(
            "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
    }

private:
    void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
    }

    void callbackResetCounter(const ResetCounter::Request::SharedPtr request, 
                              const ResetCounter::Response::SharedPtr response)
    {
        if (request->reset_value < 0) {
            response->success = false;
            response->message = "Cannot reset counter to a negative value";
        }
        else if (request->reset_value > counter_) {
            response->success = false;
            response->message = "Reset value must be lower than current counter value";
        }
        else {
            counter_ = request->reset_value;
            RCLCPP_INFO(this->get_logger(), "Reset counter to %d", counter_);
            response->success = true;
            response->message = "Success";
        }
    }

    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
    rclcpp::Service<ResetCounter>::SharedPtr reset_counter_service_;
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
```bash
$ ros2 run my_cpp_pkg number_counter
[INFO] [number_counter]: Number Counter has been started.
```

## Writing a service client
### Writing a Python service client
### Writing a C++ service client