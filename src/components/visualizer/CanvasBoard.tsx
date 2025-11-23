import { useEffect, useRef } from 'react';
import { useLessonStore } from '../../store/useLessonStore';

export const CanvasBoard = () => {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const { robotState } = useLessonStore();

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // Handle resizing
        const resizeCanvas = () => {
            const parent = canvas.parentElement;
            if (parent) {
                canvas.width = parent.clientWidth;
                canvas.height = parent.clientHeight;
            }
            draw(ctx, canvas.width, canvas.height);
        };

        window.addEventListener('resize', resizeCanvas);
        resizeCanvas(); // Initial draw

        return () => window.removeEventListener('resize', resizeCanvas);
    }, [robotState]); // Re-draw when robotState changes

    const draw = (ctx: CanvasRenderingContext2D, width: number, height: number) => {
        // Clear canvas
        ctx.fillStyle = '#0f172a'; // slate-950
        ctx.fillRect(0, 0, width, height);

        // Draw Grid
        ctx.strokeStyle = '#1e293b'; // slate-800
        ctx.lineWidth = 1;
        const gridSize = 40;
        const offsetX = width / 2;
        const offsetY = height / 2;

        // Vertical lines
        for (let x = offsetX % gridSize; x < width; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
            ctx.stroke();
        }

        // Horizontal lines
        for (let y = offsetY % gridSize; y < height; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
            ctx.stroke();
        }

        // Draw Origin Axes
        ctx.strokeStyle = '#334155'; // slate-700
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(offsetX, 0);
        ctx.lineTo(offsetX, height);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(0, offsetY);
        ctx.lineTo(width, offsetY);
        ctx.stroke();

        // Draw Robot (Turtle)
        const robotX = offsetX + robotState.position.x * gridSize;
        const robotY = offsetY - robotState.position.y * gridSize; // Invert Y for standard coord system

        ctx.save();
        ctx.translate(robotX, robotY);

        // Draw Turtle Body
        ctx.fillStyle = '#06b6d4'; // cyan-500
        ctx.beginPath();
        ctx.arc(0, 0, 15, 0, Math.PI * 2);
        ctx.fill();

        // Draw Direction Indicator (Head)
        ctx.fillStyle = '#ecfeff'; // cyan-50
        ctx.beginPath();
        ctx.arc(10, 0, 5, 0, Math.PI * 2);
        ctx.fill();

        ctx.restore();

        // Draw Active Nodes (Visual representation)
        if (robotState.activeNodes.length > 0) {
            ctx.font = '12px monospace';
            ctx.fillStyle = '#94a3b8'; // slate-400
            ctx.textAlign = 'left';
            ctx.fillText(`Active Nodes: ${robotState.activeNodes.join(', ')}`, 10, 20);
        }

        // Draw Active Topics
        if (robotState.activeTopics.length > 0) {
            ctx.font = '12px monospace';
            ctx.fillStyle = '#94a3b8'; // slate-400
            ctx.textAlign = 'left';
            ctx.fillText(`Active Topics: ${robotState.activeTopics.join(', ')}`, 10, 40);
        }
    };

    return (
        <div className="w-full h-full bg-slate-950 relative overflow-hidden">
            <canvas ref={canvasRef} className="block" />
            {/* Overlay UI elements could go here */}
        </div>
    );
};
