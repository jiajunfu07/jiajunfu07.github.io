# Writing and Building a ROS 2 Node
**Table of Contents**
- [Writing and Building a ROS 2 Node](#writing-and-building-a-ros-2-node)
  - [Creating a workspace](#creating-a-workspace)
    - [Building the workspce](#building-the-workspce)
  - [Create a package](#create-a-package)
    - [Python package](#python-package)
    - [C++ package](#c-package)
    - [Building a package](#building-a-package)
  - [Create a Python node](#create-a-python-node)
    - [Build the node](#build-the-node)
  - [Creating a C++ node](#creating-a-c-node)
    - [Building and running the node](#building-and-running-the-node)
  - [Node template for Python and C++ nodes](#node-template-for-python-and-c-nodes)
  - [Introspection](#introspection)
    - [ros2 node command line](#ros2-node-command-line)
    - [Changing the node name at runtime](#changing-the-node-name-at-runtime)
  - [Summary](#summary)
## Creating a workspace
When you make progress and start to work on several applications, the best practice is to name each workspace with the name of the application or robot. \
For example, if you create a workspace for a robot named `ABC V3`, then you can name it `abc_v3_ws`\
Open a terminal, navigate to the home directory, and create the workspace.
```
$ cd ~
$ mkdir ros2_ws
```
Then, enter the workspace and create a new directory named `src`
```
$ cd ros2_ws/
$ mkdir src
```
### Building the workspce
Navigate to the workspace root directory, and run `colcon build` command. `colcon` is the build system in ROS2. It was installed by installing the `ros-dev-tools` packages.
```
$ cd ~/ros2_ws/
$ colon build
Summary: 0 packages finished [0.73s]
```
Currently, no packages were built, but we can see three new directories
```
$ ls
build   install   log   src
```
* `build`: the directory will contain the intermediate files required for the overall build.
* `log`: the logs for each build.
* `install`: all the nodes will be installed after building the workspace

If you make a mistake and run `colcon build` from another directory (let's say, from the `src` directory of the workspace, or inside a package), simply remove the new `install`, `build`, and `log` directories that were created in the wrong place. Then go back to the workspace root directory and build again.\
If you navigate inside the newly created `install` directory you can see a `setup.bash` file. **Every time you build your workspace, you have to source it.**

```
$ source ~/ros2_ws/install/setup.bash
```
We can add it into `.bashrc` as below, so we don't need to source it everytime we open a new terminal.
```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Create a package
Any node you create will exist within a package. Hence, to create a node, you first have to create a package (inside your workspace). The architecutre of a Python package is quite different from a C++ package.
### Python package
1. `ros2 pkg create <pkg_name>`: This is the minimum you need to write.
2. You can specify a build type with `--build_type <build_type>`. For a Python package, we need to use ament_python
3. You can also specify some optional dependencies with `--dependencies <list_of_dependencies_separated_with_spaces>`. It's always possible to add dependencies later in the package.
```
$ cd ~/ros2_ws/src/
$ ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
```
/home/<user>/ros2_ws/src/my_py_pkg
├── my_py_pkg
|   └── __init__.py
├── package.xml
├── source
|   └── my_py_pkg
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```
* `my_py_pkg`: Another directory with the same name. This directory already contains an `__init__.py` file. This is where we will create our Python nodes.
* `package.xml`: Every ROS 2 package must contain this file. We will use it to provide more information about the packages as well as dependencies.
* `setup.py`: This is where you will write the instructions to build and install your Python nodes.

### C++ package
```
$ cd ~/ros2_ws/src/
$ ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```
```
/home/<user>/ros2_ws/src/my_cpp_pkg
├── CMakeLists.txt
├── include
|   └── my_cpp_pkg
├── package.xml
└── src
```
* `CMakeLists.txt`: This will be used to provide instructions on how to compile your C++ nodes, create libraries, and so on.
* `include` directory: In a C++ project, you may split your code into implementation files(.cpp extension) and header files(.hpp extension). This directory is used for the header files.
* `package.xml`: The same functionality as the one in python package.
* `src` directory: This is where you will write your C++ nodes (.cpp files).

### Building a package
To build the packages, go back to the root of your ROS 2 workspace and run `colon build`. \
```
$ cd ~/ros2_ws/
$ colon build
Starting >>> my_cpp_pkg
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.60s]
Finished <<< my_cpp_pkg [3.46s]
Summary: 2 packages finished [3.72s]
```
The important thing to notice is the line: `Finished <<< <package name> [time]`. This means that the package was correctly built. \
After you build any package, you also have to source your workspace so that the environment is aware of the new changes. \
To build only a spefific package, you can use the `--packages-select` option, followed by the name of the package.
```
$ colon build --packages-select my_py_pkg
```

## Create a Python node
To create a node:
1. Create a file for the node.
2. Write the node.
3. Build the package in which the node exist.
4. Run the node to test it.
```
$ cd ~/ros2_ws/src/my_py_pkg/my_py_pkg/
$ touch my_first_node.py
$ chmod +x my_first_node.py
```
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
   def __init__(self):
       super().__init__('my_node_name')
       self.counter_ = 0
       self.timer_ = self.create_timer(1.0, self.print_hello)

   def print_hello(self):
       self.get_logger().info("Hello " + str(self.counter_))
       self.counter_ += 1

def main(args=None):
   rclpy.init(args=args)
   node = MyCustomNode()
   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == '__main__':
   main()
```
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
```
We first import `rclpy`, the Python library for ROS 2. Inside this library, we can get the `Node` class.
```
class MyCustomNode(Node):
   def __init__(self):
       super().__init__('my_node_name')
```
We then create a new class that inherits from the rclpy Node class. In this class, make sure you call the parent constructor with `super()`. This is also where you will specify the node name.
```
def main(args=None):
   rclpy.init(args=args)
   node = MyCustomNode()
   rclpy.spin(node)
   rclpy.shutdown()
```
In the `main()` function, we preform:
1. Initialize ROS2 communication with `rclpy.init()`.
2. Create an object from the `MyCustomNode` class we wrote before.
3. Make the node spin.
4. To create the timer we use the `create_timer()` method form the `Node` class. We need to give two arguments: the rate at which we want to call the function(float number), and the callback function. Note that the callback function should be specified without any parenthesis.
5. After the node is killed, shut down ROS 2 communications with `rclpy.shutdown()`.

```
if __name__ == '__main__':
   main()
```
The last one is a pure Python thing and has nothing to do with ROS2. It just means that if you run the Python script directly, the `main()` fuction will be called. \
As the `MyCustonmNode` class inherits from the `Node` class, we get access to all the ROS 2 functionalities for nodes. For example, we get the `get_logger()` method from Node. Then, The `info()` method, we can print a log with info level.

### Build the node
We can test the code just by running it in the terminal(`$ python3 my_first_node.py`). However, what we want to do is actually install the file in our workspace, so we can start the node with ros2 run, and later on, from a launch file. \
We usually use the word "build", because to install a Python node, we have to run `colcon build`.
To build(install) the node, we need to do one more thing in the package. Open the `setup.py` file from the `my_py_pkg` package. Locate `entry_points` and `'console_scripts'` at the end of the file. For each node we want to build, we have to add one line inside the `'console_scripts'` array:
```
    entry_points={
        'console_scripts': [
            "test_node = my_py_pkg.my_first_node:main"
        ],
    },
```
Here is the syntax
```
<executable_name> = <package_name>.<file_name>:<function_name>
```
* First, choose an executable name. This will be the name you use with `ros2 run <pkg_name> <executable_name>`
* For the filename, skip the `.py` extension.
* The function name is `main`, as we have created a `main()` function in the code.
* If you want to add another executable for another node, don't forget to add a comma between each executable and place one executable per line.
* Node name, filename, executable name sometimes could be the same.
```
$ cd ~/ros2_ws/
$ colon build --packages-select my_py_pkg
```
On top of --packages-select <pkg name>, you can add the --symlink-install option, so you won't have to build the package every time you modify your Python nodes; for example, `$ colon build --packages-select my_py_pkg --symlink-install`. This only works for Python packages.

## Creating a C++ node
```
#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node
{
public:
   MyCustomNode() : Node("my_node_name"), counter_(0)
   {
       timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MyCustomNode::print_hello, this));
   }

   void print_hello()
   {
       RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
       counter_++;
   }
private:
   int counter_;
   rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MyCustomNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
```
We first include `rclcpp`, the C++ library for ROS 2. This library contains the `rclcpp::Node` class. \
We create a class that inherits from the `Node` class. From this `Node` class, we will be able to access all the ROS 2 functionalities: logger, timer, and so on. As you can see, we also specify the node name in the constructor.
```
#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node
{
public:
   MyCustomNode() : Node("my_node_name"), counter_(0)
   {
   }
private:
};
```
In the `main()` function, we do exactly the same thing as for Python.
1. Initialize ROS 2 comminications with `rclcpp::init()`
2. Create a node object from your newly written class. As you don't create an object directly, but a shared pointer to that object, you have to use `std::make_shared<>()` function. In ROS 2 and C++, almost everything you create will be a smart pointer (shared, unique, and so on).
3. Make the node spin with `rclcpp::spin()`
4. After the node is killed, shut down ROS 2 communications with `rclcpp::shutdown()`
```
int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MyCustomNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
```
As you can see, once again, the node is not the program itself. The node is just an object that we create in the `main()` function. This is a very important concept to understand when you start working with ROS 2 and C++. You can have multiple nodes in the same program, and you can also have multiple programs running at the same time, each with its own nodes.
```
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyCustomNode::print_hello, this));
```
For the timer, we have to create a class attribute. As you can see, we also create a shared pointer here: `rclcpp::TimerBase::SharedPtr timer_;`. \
We use the `create_wall_timer()` method to create a timer. `this->` is not required, but I have added it to emphasize that we are using the `create_wall_timer()` method from the `Node` class. \
The first argument is the rate at which we want to call the callback function, and the second argument is the callback function itself. We use `std::bind()` to bind the callback function to the class method.
### Building and running the node
To build the C++ node, we need to add some lines to the `CMakeLists.txt` file of the package. First, we need to find the required packages with `find_package()`. Then, we need to add an executable for our node with `add_executable()`, and link it with the required libraries with `ament_target_dependencies()`.
```
# find dependencies
... ...
find_package(rclcpp REQUIRED)
```
```
add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME}
)
```
To build a C++ node, we need to do three things in the `CMakeLists.txt` file:
1. Add a new executable with `add_executable()`. Here, you have to choose a name for the executable (the one that will be used with `ros2 run <pkg_name> <executable_name>`), and provide the path to the source file.
2. Link the executable with the required libraries with `ament_target_dependencies()`. Here, you have to provide the executable name and the list of libraries.
3. Finally, install the executable with the `install()` function. so that we can find it when we use `ros2 run`. Here, you have to provide the executable name and the destination path. The destination path should always be `lib/${PROJECT_NAME}`.
```
$ cd ~/ros2_ws/
$ colon build --packages-select my_cpp_pkg
```
After building the package, don't forget to source the workspace again.
```
$ source ~/ros2_ws/install/setup.bash
```
Now, you can run the node with `ros2 run <pkg_name> <executable_name>`
```
$ ros2 run my_cpp_pkg test_node
[INFO] [my_node_name]: Hello 0
[INFO] [my_node_name]: Hello 1
[INFO] [my_node_name]: Hello 2
...
```

## Node template for Python and C++ nodes
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node): # MODIFY NAME
   def __init__(self):
       super().__init__('node_name') # MODIFY NAME
       # Your code here

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
```
#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
   MyCustomNode() : Node("node_name") // MODIFY NAME
   {
       // Your code here
   }
};
private:
};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
```
## Introspection
### ros2 node command line
Lists all the currently running nodes
```
$ ros2 node list
/my_node_name
```
Provides information about a specific node, including its publishers, subscribers, services, and parameters.
```
$ ros2 node info /my_node_name
/my_node_name
  Subscribers:
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Services:
    /my_node_name/get_parameters: rcl_interfaces/srv/GetParameters
    /my_node_name/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_node_name/list_parameters: rcl_interfaces/srv/ListParameters
    /my_node_name/set_parameters: rcl_interfaces/srv/SetParameters
    /my_node_name/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:
  Actions Servers:
  Actions Clients:
```
### Changing the node name at runtime
To add any additional arguments when you run a node, you can use the `--ros-args` flag. \
To rename the node, add `-r __node:=<new_node_name>` when you run the node. `-r` means remap. you could also use `--remap` instead of `-r`.
```
$ ros2 run my_py_pkg my_first_node.py --ros-args -r __node:=new_node_name
```
When running multiple nodes, you should make sure that each node has a unique name. Having two nodes with the same name can lead to unexpected issues.

## Summary
1. Create and set up a ROS 2 workspace.
2. In the workspace, you can create several packages (Python or C++) that represent sub-parts of your application.
3. Create a file inside the package directory.
4. Write the node code (using the templates above).
5. Set the build instructions in `setup.py` for Python nodes or `CMakeLists.txt` for C++ nodes.
6. Build the package from the workspace root directory.