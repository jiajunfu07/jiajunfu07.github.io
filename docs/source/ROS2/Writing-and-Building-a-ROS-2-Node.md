# Writing and Building a ROS 2 Node

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
