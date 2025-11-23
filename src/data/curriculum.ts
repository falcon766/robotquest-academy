import type { Lesson } from '../types';

export interface Module {
    id: string;
    title: string;
    lessons: Lesson[];
}

export interface Course {
    id: string;
    title: string;
    description: string;
    modules: Module[];
}

export const curriculum: Course[] = [
    {
        id: 'intro_ros2',
        title: 'Introduction to ROS 2',
        description: 'Zero to Hero: Master the basics of Robot Operating System 2',
        modules: [
            {
                id: 'module_1',
                title: 'Module 1: Linux & Terminal Basics',
                lessons: [
                    {
                        id: 'lesson_1',
                        title: '1. Introduction to the Terminal',
                        contentMarkdown: `
## What is the Terminal?

The terminal (also called command line or shell) is a text-based interface for interacting with your computer. Unlike clicking buttons in a graphical interface, you type commands to tell the computer what to do.

In robotics, the terminal is essential because:
- **Efficiency**: Execute complex operations with a single command
- **Automation**: Create scripts to run repetitive tasks
- **Remote Access**: Control robots from anywhere without a screen
- **Debugging**: See detailed system logs and error messages

## Your First Command: echo

The \`echo\` command is one of the simplest commands in Linux. It prints text to the terminal.

**Syntax:**
\`echo "Your text here"\`

**Why start with echo?**
- It's safe (won't change any files or settings)
- You get instant visual feedback
- It teaches command syntax (command + arguments)

## Try It Yourself

Type the following command in the terminal on the right:

\`echo "Hello ROS"\`

**What happens?**
The terminal will display "Hello ROS" back to you. This confirms that:
1. Your terminal is working
2. You can execute commands
3. You understand basic command syntax

In ROS, you'll use similar commands to send messages between robot components!
                        `,
                        initialCode: '',
                        expectedCommand: 'echo "Hello ROS"',
                        successMessage: 'Excellent! You just executed your first command. The terminal echoed your message back. In ROS, nodes communicate by echoing messages like this!',
                        xpReward: 10,
                    },
                    {
                        id: 'lesson_2',
                        title: '2. Understanding File Systems',
                        contentMarkdown: `
## What is a File System?

A file system is how your computer organizes and stores data. Think of it like a filing cabinet with folders and documents.

In Linux (which ROS uses), everything is organized in a **tree structure** starting from the root directory \`/\`.

## The pwd Command

\`pwd\` stands for **"Print Working Directory"**. It shows you where you currently are in the file system.

**Why is this important in robotics?**
- ROS packages are stored in specific directories
- Launch files need absolute paths to work
- Logs and configuration files have standard locations
- Knowing your location prevents path errors

## Try It Yourself

Type:

\`pwd\`

**What you'll see:**
The terminal will print your current directory path (e.g., \`/home/user/workspace\`).

**In ROS, you'll often need to:**
- Navigate to package directories
- Source workspace setup files from specific paths
- Reference configuration files with absolute paths

Understanding where you are in the file system is the foundation for everything else!
                        `,
                        initialCode: '',
                        expectedCommand: 'pwd',
                        successMessage: 'Great! You can now see your current location. This is crucial when working with ROS workspaces and packages.',
                        xpReward: 15,
                    },
                    {
                        id: 'lesson_3',
                        title: '3. Listing Files (ls)',
                        contentMarkdown: `
## Seeing What's Inside

Now that you know *where* you are, you need to see *what* is in the directory.

## The ls Command

\`ls\` stands for **List**. It lists all files and folders in the current directory.

**Common Options:**
- \`ls\`: Simple list
- \`ls -l\`: Long format (shows permissions, size, date)
- \`ls -a\`: All files (including hidden files starting with .)

## Try It Yourself

Type:

\`ls\`

You should see a list of files. In our simulation, you might see folders like \`src\`, \`build\`, or \`install\` if you are in a ROS workspace.
                        `,
                        initialCode: '',
                        expectedCommand: 'ls',
                        successMessage: 'Nice! You can now inspect directories. In ROS, you will use this constantly to check if packages are built or if bag files are saved.',
                        xpReward: 15,
                    },
                    {
                        id: 'lesson_4',
                        title: '4. Changing Directories (cd)',
                        contentMarkdown: `
## Moving Around

To navigate the file tree, you use the \`cd\` command.

## The cd Command

\`cd\` stands for **Change Directory**.

**Syntax:**
- \`cd foldername\`: Go into a folder
- \`cd ..\`: Go up one level (to the parent folder)
- \`cd ~\`: Go to your home directory
- \`cd /\`: Go to the root directory

## Try It Yourself

Let's try to move into the \`src\` directory (common in ROS workspaces).

Type:

\`cd src\`

(If it fails, try \`ls\` first to see what folders exist, then \`cd\` into one of them!)
                        `,
                        initialCode: '',
                        expectedCommand: 'cd src',
                        successMessage: 'You moved! Navigation is key. In ROS, you will jump between your workspace source, build folders, and system install paths frequently.',
                        xpReward: 20,
                    },
                    {
                        id: 'lesson_5',
                        title: '5. Creating Directories (mkdir)',
                        contentMarkdown: `
## Organizing Your Workspace

In ROS 2, you need to create specific folder structures for your packages.

## The mkdir Command

\`mkdir\` stands for **Make Directory**.

**Syntax:**
\`mkdir <directory_name>\`

## Try It Yourself

Create a new directory called \`my_package\`.

Type:

\`mkdir my_package\`
                        `,
                        initialCode: '',
                        expectedCommand: 'mkdir my_package',
                        successMessage: 'Directory created! You will use this to create new ROS packages and organize your launch files.',
                        xpReward: 20,
                    },
                    {
                        id: 'lesson_6',
                        title: '6. Creating Files (touch)',
                        contentMarkdown: `
## Creating Empty Files

Sometimes you need to create a placeholder file, like a Python script or a launch file.

## The touch Command

\`touch\` creates an empty file.

**Syntax:**
\`touch <filename>\`

## Try It Yourself

Create a file named \`node.py\`.

Type:

\`touch node.py\`
                        `,
                        initialCode: '',
                        expectedCommand: 'touch node.py',
                        successMessage: 'File created! In the real world, you would now open this file in a code editor to write your ROS node.',
                        xpReward: 20,
                    }
                ]
            },
            {
                id: 'module_2',
                title: 'Module 2: ROS 2 Concepts',
                lessons: [
                    {
                        id: 'lesson_2_1',
                        title: '1. Sourcing the Setup File',
                        contentMarkdown: `
## The Most Important ROS Command

Before you can run any ROS 2 command, you must **source** the setup file. This sets up your environment variables so the terminal knows where to find ROS commands.

## The source Command

**Syntax:**
\`source /opt/ros/humble/setup.bash\` (for global ROS)
\`source install/setup.bash\` (for your local workspace)

## Try It Yourself

Let's simulate sourcing the global ROS installation.

Type:

\`source /opt/ros/humble/setup.bash\`
                        `,
                        initialCode: '',
                        expectedCommand: 'source /opt/ros/humble/setup.bash',
                        successMessage: 'Environment sourced! If you ever see "command not found: ros2", you probably forgot this step.',
                        xpReward: 25,
                    },
                    {
                        id: 'lesson_2_2',
                        title: '2. What is a Node?',
                        contentMarkdown: `
## The Brain Cells of Robots

In ROS 2, a **Node** is a process that performs computation. A robot is essentially a collection of nodes communicating with each other.

**Examples of Nodes:**
- A node controlling a laser driver
- A node calculating path planning
- A node processing camera images
- A node controlling wheel motors

## Running a Node

In the terminal, you typically run a node using the \`ros2 run\` command.

**Syntax:**
\`ros2 run <package_name> <executable_name>\`

## Try It Yourself

Let's simulate running a simple "talker" node.

Type:

\`ros2 run demo_nodes_cpp talker\`
                        `,
                        initialCode: '',
                        expectedCommand: 'ros2 run demo_nodes_cpp talker',
                        successMessage: 'You started a node! This node publishes "Hello World" messages. You are now running actual ROS 2 code!',
                        xpReward: 25,
                    },
                    {
                        id: 'lesson_2_3',
                        title: '3. Listing Active Nodes',
                        contentMarkdown: `
## Checking What's Running

When debugging a robot, you often need to see which nodes are currently active.

## The ros2 node list Command

**Syntax:**
\`ros2 node list\`

## Try It Yourself

See what nodes are running (including the one you just started!).

Type:

\`ros2 node list\`
                        `,
                        initialCode: '',
                        expectedCommand: 'ros2 node list',
                        successMessage: 'Correct! You should see /talker in the list. This confirms your node is alive and registered with the ROS graph.',
                        xpReward: 25,
                    }
                ]
            }
        ]
    },
    {
        id: 'module_3',
        title: 'Module 3: Visualization & Simulation',
        description: 'See what the robot sees',
        modules: [
            {
                id: 'module_3_1',
                title: '1. Introduction to Turtlesim',
                lessons: [
                    {
                        id: 'lesson_3_1',
                        title: '1. Hello World of Robotics',
                        contentMarkdown: `
## What is that Chart on the Left?

That "chart" is actually a 2D simulation window! In ROS, visualization is critical because you can't always see what the robot is thinking just by looking at the hardware.

## Meet Turtlesim

**Turtlesim** is the classic "Hello World" of ROS. It's a simple 2D simulator where you control a turtle. It teaches you about nodes, topics, and services without needing a complex physics engine.

## Try It Yourself

Let's wake up the turtle! Run the turtlesim node.

Type:

\`ros2 run turtlesim turtlesim_node\`

**Watch the panel on the left!** You should see a blue turtle appear in the center.
                        `,
                        initialCode: '',
                        expectedCommand: 'ros2 run turtlesim turtlesim_node',
                        successMessage: 'The turtle is alive! You just started a graphical node. This is your first step into robot simulation.',
                        xpReward: 30,
                    }
                ]
            }
        ]
    },
    {
        id: 'intermediate_ros2',
        title: 'Intermediate ROS 2',
        description: 'Advanced concepts: TF2, Navigation, and Perception',
        modules: [
            {
                id: 'module_int_1',
                title: 'Module 1: TF2 & Transforms',
                lessons: [
                    {
                        id: 'lesson_int_1',
                        title: '1. Introduction to TF2',
                        contentMarkdown: 'Coming soon...',
                        initialCode: '',
                        expectedCommand: 'echo "TF2"',
                        successMessage: 'Stay tuned!',
                        xpReward: 0
                    }
                ]
            }
        ]
    }
];
