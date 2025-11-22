import type { Lesson } from '../types';

interface CommandResult {
    output: string;
    success: boolean;
    action?: 'clear' | 'start_node' | 'stop_node' | 'publish_topic';
    payload?: any;
}

export const parseCommand = (input: string, currentLesson: Lesson | null): CommandResult => {
    const trimmedInput = input.trim();
    const parts = trimmedInput.split(' ');
    const command = parts[0];

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
  source    Source the ROS 2 setup file
  ros2      Run ROS 2 commands (run, topic, node)`,
            success: true,
        };
    }

    if (command === 'echo') {
        return { output: parts.slice(1).join(' '), success: true };
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
