Product Requirements Document: ROS 2 Interactive Academy

Project Name: Droid Academy
Target Platform: Web (Responsive, Desktop-first for coding)
Tech Stack: React (Vite), Tailwind CSS, Firebase (Auth/Firestore), Netlify
AI Builder Strategy: "Antigravity" / Agentic Development

1. Executive Summary

Concept: A "Brilliant.org" style interactive course for learning Robot Operating System 2 (ROS 2).
Core Value: ROS 2 is hard to install. This platform removes the setup barrier, allowing users to learn Nodes, Topics, and Services in a simulated browser terminal.
Tone: Sci-fi, playful, gamified, yet technically accurate.
Target Audience: Ages 12+ (Advanced Elementary to Young Adult).

2. Technical Architecture (The "AI Prompting" Guide)

2.1 Tech Stack & Libraries

Framework: React 18+ (Vite)

Language: TypeScript (Strict mode)

Styling: Tailwind CSS (for layout) + Framer Motion (for "juicy" gamification animations).

State Management: Zustand (Simpler for AI to manage than Redux).

Code/Terminal Editor: @monaco-editor/react (VS Code style editor) or a custom styled textarea for the terminal CLI.

Icons: lucide-react

Backend/BaaS: Firebase v9 (Modular SDK).

Auth: Google & Email/Password.

Firestore: Database for user progress.

Routing: react-router-dom

2.2 The "Mock ROS" Engine (Crucial Logic)

Since we are not running a real Linux container backend, the AI must build a Frontend Simulation Engine.

The State Machine: The app must track the state of a "Virtual Robot".

Example State: { isNodeRunning: false, activeTopics: [], robotPosition: {x:0, y:0} }

Command Parsing: When the user types ros2 run demo_nodes_cpp talker in the web terminal, the Engine does not run C++. Instead, it parses the string, checks if it matches the lesson requirement, and updates the UI to show a fake node starting.

3. User Experience & Flows

3.1 Onboarding

Landing Page: Hero section with a 3D spline or simple CSS animation of a robot. "Start Learning" button.

Auth: Firebase Login (Google/Email).

Placement: Simple question: "Total Beginner" vs "I know Python."

3.2 The Learning Interface (The Core Loop)

Layout: Split Screen (Desktop).

Left Panel (Curriculum):

Markdown-rendered lesson text.

"Objective" box (e.g., "Start the turtle node").

Progress bar for the current module.

Right Panel (Interactive):

Tab 1: Terminal: A dark-mode CLI where users type commands.

Tab 2: Visualizer (Rviz-lite): A 2D HTML5 Canvas that renders the "Robot" (e.g., a turtle or simple geometric shape) moving based on commands.

Footer:

Check Answer Button.

Console Output log.

3.3 Gamification (The "Brilliant/Duolingo" Hook)

XP System: +10 XP for reading, +50 XP for solving code challenges.

Streak Counter: Track consecutive days logged in.

Battery Life: (Optional) Instead of "Hearts", use "Battery". Incorrect answers drain battery.

Sound Effects: Success chime, Error buzzer, Typing sounds (ASMR).

4. Data Model (Schema)

Collection: users

interface User {
  uid: string;
  displayName: string;
  email: string;
  xp: number;
  streak: number;
  lastLogin: Timestamp;
  currentModuleId: string; // e.g., "module_01_nodes"
  completedLessons: string[]; // Array of IDs ["lesson_1", "lesson_2"]
  settings: {
    soundEnabled: boolean;
    theme: 'dark' | 'light';
  }
}


Collection: modules (Static Content - likely hardcoded in JSON for MVP)

interface Module {
  id: string;
  title: string; // e.g., "The Ros Graph"
  order: number;
  description: string;
  lessons: Lesson[];
}

interface Lesson {
  id: string;
  title: string;
  contentMarkdown: string;
  initialCode: string; // Pre-filled code in editor
  expectedCommand: string; // Regex or string to validate user input
  successMessage: string;
  xpReward: number;
}


5. Curriculum Outline (MVP Content)

Module 1: The Shell & The Setup

Lesson 1.1: Hello World. Typing echo "Hello ROS" in the simulator.

Lesson 1.2: Sourcing the setup (The famous source /opt/ros/humble/setup.bash).

Interaction: User must type the source command to "power on" the terminal.

Module 2: Nodes (The Brain Cells)

Lesson 2.1: What is a Node?

Lesson 2.2: Running a Node.

Task: Run ros2 run demo_nodes_cpp talker.

Visual: A box appears in the Visualizer labeled "Talker".

Module 3: Topics (The Conversation)

Lesson 3.1: Pub/Sub Theory.

Lesson 3.2: Rqt Graph.

Task: Run a listener node.

Visual: An arrow connects "Talker" to "Listener" in the canvas.

Module 4: Turtlesim (The Classic)

Lesson 4.1: Spawning the Turtle.

Lesson 4.2: Moving the Turtle via CLI.

Task: ros2 topic pub /turtle1/cmd_vel ...

Visual: The 2D Turtle moves on the HTML canvas.

6. Implementation Steps for AI Agent

Phase 1: Skeleton & Auth

Initialize React + Vite + Tailwind.

Set up Firebase Auth context.

Create the Main Layout (Navbar, Sidebar, Content Area).

Phase 2: The Terminal Component

Build a Terminal component that accepts text input.

Create a CommandParser utility function.

Logic: If input == expected_command, return success + XP. Else return standard bash error.

Implement the LessonProvider (Zustand store) to track which lesson is active.

Phase 3: The Visualizer

Create a CanvasBoard component.

Connect Terminal commands to Canvas state.

Logic: If user sends a "move" command, update X/Y coordinates in the store, trigger re-render of Canvas.

Phase 4: Gamification & Save

Connect "Task Complete" event to Firestore updateDoc (increment XP).

Add confetti effect on lesson completion.

Build the User Dashboard (Progress map).

7. Design System Guidelines

Color Palette:

Primary: Cyber Blue (#00f0ff) - for active elements.

Secondary: ROS Purple (#5e35b1) - for branding.

Background: Deep Space (#0f172a) - easier on eyes for coding.

Text: Slate 200.

Typography:

Headings: 'Orbitron' or 'Inter' (Bold, modern).

Code: 'JetBrains Mono' or 'Fira Code' (Essential for readability).

UI Elements:

Buttons should have a subtle "glow" effect.

Containers should have "glassmorphism" (backdrop-blur) borders.

8. Specific Prompts for Antigravity/Cursor

"Generate a React component for a simulated Linux terminal. It should maintain a history of commands, support arrow key navigation for history, and parse specific 'ros2' commands defined in a config object."

"Create a Zustand store to manage the 'Virtual Robot' state, including x/y position, active nodes list, and current battery level."

"Write a Firebase service to handle user XP updates, ensuring we use Firestore atomic increments to prevent race conditions."