import React, { useState, useRef, useEffect } from 'react';
import { useLessonStore } from '../../store/useLessonStore';
import { parseCommand } from '../../utils/commandParser';
import { useAuth } from '../../contexts/AuthContext';

export const Terminal = () => {
    const { terminalHistory, addLog, currentLesson, updateRobotState, clearLogs, completeLesson } = useLessonStore();
    const { currentUser } = useAuth();
    const [input, setInput] = useState('');
    const [historyIndex, setHistoryIndex] = useState(-1);
    const inputRef = useRef<HTMLInputElement>(null);
    const bottomRef = useRef<HTMLDivElement>(null);

    // Auto-scroll to bottom
    useEffect(() => {
        bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [terminalHistory]);

    // Focus input on click
    const handleContainerClick = () => {
        inputRef.current?.focus();
    };

    const handleKeyDown = (e: React.KeyboardEvent) => {
        if (e.key === 'Enter') {
            e.preventDefault();
            if (!input.trim()) return;

            // Add input to log
            addLog('input', input);

            // Parse command
            const result = parseCommand(input, currentLesson);

            // Handle actions
            if (result.action === 'clear') {
                clearLogs();
            } else {
                if (result.output) {
                    addLog(result.success ? 'output' : 'error', result.output);
                }

                if (result.success && currentLesson && currentUser) {
                    // Check if it was a lesson completion (simple check: if parseCommand returned success for a lesson)
                    // parseCommand returns success: true if it matched the lesson expectation.
                    // We should probably have a more explicit flag in CommandResult, but this works for now.
                    completeLesson(currentUser.uid);
                }

                if (result.action === 'start_node') {
                    updateRobotState({ isNodeRunning: true, activeNodes: [result.payload.executable] });
                }
                if (result.action === 'publish_topic') {
                    updateRobotState({ activeTopics: [result.payload.topic] });
                }
            }

            setInput('');
            setHistoryIndex(-1);
        } else if (e.key === 'ArrowUp') {
            e.preventDefault();
            // Simple history navigation (could be improved)
            const inputs = terminalHistory.filter(h => h.type === 'input');
            if (inputs.length > 0) {
                const newIndex = historyIndex === -1 ? inputs.length - 1 : Math.max(0, historyIndex - 1);
                setHistoryIndex(newIndex);
                setInput(inputs[newIndex].content);
            }
        } else if (e.key === 'ArrowDown') {
            e.preventDefault();
            const inputs = terminalHistory.filter(h => h.type === 'input');
            if (historyIndex !== -1) {
                const newIndex = Math.min(inputs.length - 1, historyIndex + 1);
                setHistoryIndex(newIndex);
                setInput(inputs[newIndex].content);

                if (newIndex === inputs.length - 1) {
                    // If at end, maybe clear? For now just stay at last
                }
            }
        }
    };

    return (
        <div
            className="bg-slate-950 font-mono text-sm p-4 h-full overflow-hidden flex flex-col border-t border-slate-800"
            onClick={handleContainerClick}
        >
            <div className="flex-1 overflow-y-auto space-y-1 custom-scrollbar">
                {terminalHistory.map((entry) => (
                    <div key={entry.id} className={`${entry.type === 'error' ? 'text-red-400' : entry.type === 'input' ? 'text-slate-300' : 'text-slate-400'}`}>
                        {entry.type === 'input' && <span className="text-green-500 mr-2">$</span>}
                        <span className="whitespace-pre-wrap">{entry.content}</span>
                    </div>
                ))}
                <div ref={bottomRef} />
            </div>

            <div className="flex items-center mt-2">
                <span className="text-green-500 mr-2">$</span>
                <input
                    ref={inputRef}
                    type="text"
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    onKeyDown={handleKeyDown}
                    className="flex-1 bg-transparent border-none outline-none text-slate-200 placeholder-slate-600"
                    placeholder="Type a command..."
                    autoComplete="off"
                    autoFocus
                />
            </div>
        </div>
    );
};
