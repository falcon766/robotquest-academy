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
                    }
                ]
            },
            {
                id: 'module_2',
                title: 'Module 2: ROS 2 Concepts',
                lessons: [
                    {
                        id: 'lesson_2_1',
                        title: '1. What is a Node?',
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
