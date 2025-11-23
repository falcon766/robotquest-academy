
import { useLessonStore } from '../../store/useLessonStore';
import { Terminal, Share2, MessageSquare } from 'lucide-react';

export const Sidebar = () => {
    const { setCurrentLesson } = useLessonStore();

    const startLesson1 = () => {
        setCurrentLesson({
            id: 'lesson_1',
            title: 'Introduction to the Terminal',
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
        });
    };

    const startLesson2 = () => {
        setCurrentLesson({
            id: 'lesson_2',
            title: 'Understanding File Systems',
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
        });
    };

    return (
        <aside className="w-64 bg-black border-r border-slate-800 h-screen p-4 hidden md:flex flex-col">
            <div className="space-y-6">
                <div className="text-slate-500 text-xs font-bold uppercase tracking-widest px-2">Curriculum</div>
                <div className="space-y-1">
                    <button
                        onClick={startLesson1}
                        className="w-full flex items-center gap-3 px-3 py-2.5 bg-slate-900 text-slate-200 rounded-lg border border-slate-800 hover:border-orange-500/50 hover:bg-slate-800 transition-all group text-left"
                    >
                        <Terminal size={18} className="text-slate-500 group-hover:text-orange-400 transition-colors" />
                        <span className="text-sm font-medium">Intro to Terminal</span>
                    </button>

                    <button
                        onClick={startLesson2}
                        className="w-full flex items-center gap-3 px-3 py-2.5 bg-slate-900 text-slate-200 rounded-lg border border-slate-800 hover:border-orange-500/50 hover:bg-slate-800 transition-all group text-left"
                    >
                        <Share2 size={18} className="text-slate-500 group-hover:text-orange-400 transition-colors" />
                        <span className="text-sm font-medium">File Systems</span>
                    </button>

                    <button className="w-full flex items-center gap-3 px-3 py-2.5 text-slate-500 rounded-lg border border-transparent hover:bg-slate-900/50 transition-all text-left cursor-not-allowed opacity-60">
                        <MessageSquare size={18} />
                        <span className="text-sm font-medium">ROS Topics</span>
                    </button>
                </div>
            </div>
        </aside>
    );
};
