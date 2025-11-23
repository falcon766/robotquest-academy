export interface Lesson {
    id: string;
    title: string;
    contentMarkdown: string;
    initialCode: string;
    expectedCommand: string | RegExp;
    successMessage: string;
    xpReward: number;
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
