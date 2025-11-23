import type { Lesson } from '../types';
import { useLessonStore } from '../store/useLessonStore';

interface CommandResult {
    output: string;
    success: boolean;
    action?: 'clear' | 'start_node' | 'stop_node' | 'publish_topic';
    payload?: any;
}

export const parseCommand = (input: string, currentLesson: Lesson | null): CommandResult => {
    const trimmedInput = input.trim();
    // Handle quoted strings in arguments (simple version)
    // For now, simple split by space, but we should respect quotes for echo
    const parts = trimmedInput.match(/(?:[^\s"]+|"[^"]*")+/g) || [];
    const command = parts[0];

    // Get store actions directly (non-reactive way is fine for this utility)
    const store = useLessonStore.getState();

    if (!trimmedInput) {
        return { output: '', success: true };
    }

    // 1. Handle System Commands
    if (command === 'clear') {
        return { output: '', success: true, action: 'clear' };
    }

    if (command === 'help') {
        return {
            output: `Available commands:
  help      Show this help message
  clear     Clear the terminal
  echo      Print text to the terminal
  ls        List directory contents
  cd        Change directory
  pwd       Print working directory
  mkdir     Create directory
  touch     Create file
  rm        Remove file or directory
  cat       Read file content
  source    Source the ROS 2 setup file
  ros2      Run ROS 2 commands (run, topic, node)`,
            success: true,
        };
    }

    if (command === 'echo') {
        const text = parts.slice(1).join(' ').replace(/"/g, '');
        return { output: text, success: true };
    }

    // File System Commands
    if (command === 'ls') {
        const output = store.listFiles();
        return { output, success: true };
    }

    if (command === 'pwd') {
        const path = '/' + store.currentPath.join('/');
        return { output: path, success: true };
    }

    if (command === 'cd') {
        const path = parts[1];
        if (!path) {
            // cd without args goes home
            store.changeDirectory('~');
            return { output: '', success: true };
        }
        const error = store.changeDirectory(path);
        if (error) {
            return { output: error, success: false };
        }
        return { output: '', success: true };
    }

    if (command === 'mkdir') {
        const name = parts[1];
        if (!name) return { output: 'mkdir: missing operand', success: false };
        const error = store.createDirectory(name);
        if (error) return { output: error, success: false };
        return { output: '', success: true };
    }

    if (command === 'touch') {
        const name = parts[1];
        if (!name) return { output: 'touch: missing operand', success: false };
        const error = store.createFile(name);
        if (error) return { output: error, success: false };
        return { output: '', success: true };
    }

    if (command === 'cat') {
        const name = parts[1];
        if (!name) return { output: 'cat: missing operand', success: false };
        const output = store.readFile(name);
        return { output, success: true };
    }

    if (command === 'rm') {
        const name = parts[1];
        if (!name) return { output: 'rm: missing operand', success: false };
        const error = store.removeNode(name);
        if (error) return { output: error, success: false };
        return { output: '', success: true };
    }

    // 2. Handle ROS 2 Commands
    if (command === 'ros2') {
        const subCommand = parts[1];

        if (!subCommand) {
            return { output: 'ros2: missing command argument', success: false };
        }

        // ros2 run <package> <executable>
        if (subCommand === 'run') {
            const pkg = parts[2];
            const exec = parts[3];
            if (!pkg || !exec) {
                return { output: 'usage: ros2 run <package> <executable>', success: false };
            }

            // Special handling for turtlesim
            if (pkg === 'turtlesim' && exec === 'turtlesim_node') {
                return {
                    output: `[INFO] [turtlesim]: Starting turtlesim node\n[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]`,
                    success: true,
                    action: 'start_node',
                    payload: { package: pkg, executable: exec },
                };
            }

            return {
                output: `[INFO] [launch]: process[${exec}-1]: started with pid [1234]`,
                success: true,
                action: 'start_node',
                payload: { package: pkg, executable: exec },
            };
        }

        // ros2 topic list
        if (subCommand === 'topic') {
            if (parts[2] === 'list') {
                return {
                    output: '/parameter_events\n/rosout',
                    success: true,
                };
            }
            // ros2 topic pub <topic_name> <msg_type> <args>
            if (parts[2] === 'pub') {
                const topic = parts[3];
                if (!topic) {
                    return { output: 'usage: ros2 topic pub <topic_name> <msg_type> <args>', success: false };
                }
                return {
                    output: `publisher: beginning loop\npublishing to: ${topic}`,
                    success: true,
                    action: 'publish_topic',
                    payload: { topic },
                };
            }
        }

        // ros2 node list
        if (subCommand === 'node') {
            if (parts[2] === 'list') {
                return {
                    output: '/talker\n/listener', // Mocked for now
                    success: true,
                };
            }
        }

        return { output: `ros2: command not found: ${subCommand}`, success: false };
    }

    // 3. Handle Lesson Validation (if active)
    if (currentLesson) {
        const expected = currentLesson.expectedCommand;
        let isMatch = false;

        // Special handling for cd/ls/pwd to ensure they actually worked before granting success
        // But since we run the command first above, we just check if the input matches expected

        if (typeof expected === 'string') {
            isMatch = trimmedInput === expected;
        } else {
            isMatch = expected.test(trimmedInput);
        }

        if (isMatch) {
            return {
                output: currentLesson.successMessage,
                success: true,
            };
        }
    }

    // Default: Command not found
    return {
        output: `zsh: command not found: ${command}`,
        success: false,
    };
};
