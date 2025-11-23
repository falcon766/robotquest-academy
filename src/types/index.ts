export interface Lesson {
    id: string;
    title: string;
    contentMarkdown: string;
    initialCode: string;
    expectedCommand: string | RegExp;
    successMessage: string;
    xpReward: number;
    visualizationType?: 'topic' | 'service' | 'node' | 'action' | 'setup_env' | 'node_inspection' | 'node_remap' | 'parameter_set' | 'terminal_echo' | 'fs_pwd' | 'fs_ls' | 'fs_cd' | 'fs_mkdir' | 'fs_touch' | 'turtlesim_hello' | 'teleop_keys' | 'topic_cmd_vel' | 'topic_circle' | 'service_teleport' | 'service_pen' | 'service_reset' | 'service_spawn' | 'rviz_viz';
}

export interface Module {
    id: string;
    title: string;
    order: number;
    description: string;
    lessons: Lesson[];
}

export interface TerminalEntry {
    id: string;
    type: 'input' | 'output' | 'error' | 'success';
    content: string;
    timestamp: number;
}

export interface RobotState {
    isNodeRunning: boolean;
    activeNodes: string[];
    activeTopics: string[];
    position: { x: number; y: number; theta: number };
    linearVelocity: number;
    angularVelocity: number;
    battery: number;
    path: { x: number; y: number; penDown: boolean; color: string; width: number }[];
    pen: { isDown: boolean; color: string; width: number };
}

export interface UserProfile {
    uid: string;
    email: string;
    displayName: string;
    xp: number;
    level: number;
    completedLessons: string[];
    currentModuleId: string;
    streak: number;
    lastLogin: number;
}
