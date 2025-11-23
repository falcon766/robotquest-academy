import { motion } from 'framer-motion';

export type AnimationType = 'topic' | 'service' | 'node' | 'action' | 'setup_env' | 'node_inspection' | 'node_remap' | 'parameter_set' | 'terminal_echo' | 'fs_pwd' | 'fs_ls' | 'fs_cd' | 'fs_mkdir' | 'fs_touch' | 'turtlesim_hello' | 'teleop_keys' | 'topic_cmd_vel' | 'topic_circle' | 'service_teleport' | 'service_pen' | 'service_reset' | 'service_spawn' | 'rviz_viz';

interface ConceptAnimationProps {
    type: AnimationType;
}

export const ConceptAnimation = ({ type }: ConceptAnimationProps) => {
    if (type === 'topic') return <TopicAnimation />;
    if (type === 'service') return <ServiceAnimation />;
    if (type === 'node') return <NodeAnimation />;
    if (type === 'setup_env') return <SetupEnvAnimation />;
    if (type === 'node_inspection') return <NodeInspectionAnimation />;
    if (type === 'node_remap') return <NodeRemapAnimation />;
    if (type === 'parameter_set') return <ParameterSetAnimation />;
    if (type === 'terminal_echo') return <TerminalEchoAnimation />;
    if (type === 'fs_pwd') return <FsPwdAnimation />;
    if (type === 'fs_ls') return <FsLsAnimation />;
    if (type === 'fs_cd') return <FsCdAnimation />;
    if (type === 'fs_mkdir') return <FsMkdirAnimation />;
    if (type === 'fs_touch') return <FsTouchAnimation />;
    if (type === 'turtlesim_hello') return <TurtlesimHelloAnimation />;
    if (type === 'teleop_keys') return <TeleopKeysAnimation />;
    if (type === 'topic_cmd_vel') return <TopicCmdVelAnimation />;
    if (type === 'topic_circle') return <TopicCircleAnimation />;
    if (type === 'service_teleport') return <ServiceTeleportAnimation />;
    if (type === 'service_pen') return <ServicePenAnimation />;
    if (type === 'service_reset') return <ServiceResetAnimation />;
    if (type === 'service_spawn') return <ServiceSpawnAnimation />;
    if (type === 'rviz_viz') return <RvizVizAnimation />;
    return null;
};

const TerminalEchoAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="w-64 h-32 bg-slate-950 rounded border border-slate-700 p-4 font-mono text-xs relative">
                <div className="flex gap-1.5 mb-3">
                    <div className="w-2 h-2 rounded-full bg-red-500"></div>
                    <div className="w-2 h-2 rounded-full bg-yellow-500"></div>
                    <div className="w-2 h-2 rounded-full bg-green-500"></div>
                </div>
                <div className="text-slate-400">$ echo "Hello ROS"</div>
                <motion.div
                    className="text-green-400 mt-1"
                    initial={{ opacity: 0 }}
                    animate={{ opacity: [0, 1, 1, 0] }}
                    transition={{ duration: 3, repeat: Infinity, times: [0.2, 0.3, 0.8, 0.9] }}
                >
                    Hello ROS
                </motion.div>
                <motion.div
                    className="w-2 h-4 bg-slate-500 mt-1"
                    animate={{ opacity: [1, 0] }}
                    transition={{ duration: 0.8, repeat: Infinity }}
                />
            </div>
        </div>
    );
};

const FsPwdAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="flex items-center gap-2 text-sm font-mono">
                <div className="flex flex-col items-center">
                    <div className="w-12 h-10 bg-slate-700 rounded mb-1 flex items-center justify-center">/</div>
                    <span className="text-slate-500 text-[10px]">root</span>
                </div>
                <div className="h-0.5 w-8 bg-slate-600"></div>
                <div className="flex flex-col items-center">
                    <div className="w-12 h-10 bg-slate-700 rounded mb-1 flex items-center justify-center">home</div>
                    <span className="text-slate-500 text-[10px]">home</span>
                </div>
                <div className="h-0.5 w-8 bg-slate-600"></div>
                <motion.div
                    className="flex flex-col items-center"
                    animate={{ scale: [1, 1.1, 1] }}
                    transition={{ duration: 2, repeat: Infinity }}
                >
                    <div className="w-12 h-10 bg-orange-600 rounded mb-1 flex items-center justify-center text-white">user</div>
                    <span className="text-orange-400 text-[10px] font-bold">YOU ARE HERE</span>
                </motion.div>
            </div>
        </div>
    );
};

const FsLsAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="w-64 p-4">
                <div className="text-slate-400 text-xs mb-4 text-center">$ ls</div>
                <div className="grid grid-cols-4 gap-4">
                    {[0, 1, 2, 3].map(i => (
                        <motion.div
                            key={i}
                            className="flex flex-col items-center"
                            initial={{ opacity: 0, y: 10 }}
                            animate={{ opacity: 1, y: 0 }}
                            transition={{ delay: i * 0.2, duration: 0.5, repeat: Infinity, repeatDelay: 2 }}
                        >
                            <div className={`w-8 h-10 ${i % 2 === 0 ? 'bg-blue-500/20 border-blue-500' : 'bg-slate-700 border-slate-600'} border rounded mb-1`}></div>
                            <div className="w-8 h-1 bg-slate-600 rounded-full"></div>
                        </motion.div>
                    ))}
                </div>
            </div>
        </div>
    );
};

const FsCdAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="relative w-64 h-32 flex items-center justify-between px-8">
                <div className="flex flex-col items-center z-10">
                    <div className="w-12 h-10 bg-slate-700 border border-slate-500 rounded mb-1"></div>
                    <span className="text-[10px] text-slate-400">current</span>
                </div>

                <div className="flex flex-col items-center z-10">
                    <div className="w-12 h-10 bg-blue-900/50 border border-blue-500 rounded mb-1"></div>
                    <span className="text-[10px] text-blue-400">target</span>
                </div>

                <motion.div
                    className="absolute top-1/2 left-12 w-4 h-4 bg-orange-500 rounded-full z-20"
                    animate={{ x: [0, 120], y: [0, -20, 0] }}
                    transition={{ duration: 1.5, repeat: Infinity, ease: "easeInOut" }}
                />
            </div>
            <div className="absolute bottom-4 text-xs text-slate-500 font-mono">cd target</div>
        </div>
    );
};

const FsMkdirAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="flex items-center gap-4">
                <div className="w-12 h-10 bg-slate-800 border border-slate-700 rounded opacity-50"></div>
                <div className="w-12 h-10 bg-slate-800 border border-slate-700 rounded opacity-50"></div>
                <motion.div
                    className="w-12 h-10 bg-blue-600 border border-blue-400 rounded flex items-center justify-center"
                    initial={{ scale: 0, opacity: 0 }}
                    animate={{ scale: [0, 1.2, 1], opacity: 1 }}
                    transition={{ duration: 1, repeat: Infinity, repeatDelay: 1 }}
                >
                    <span className="text-lg font-bold text-white">+</span>
                </motion.div>
            </div>
            <div className="absolute bottom-4 text-xs text-slate-500 font-mono">mkdir new_folder</div>
        </div>
    );
};

const FsTouchAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="flex items-center gap-4">
                <div className="w-10 h-12 bg-slate-800 border border-slate-700 rounded opacity-50"></div>
                <motion.div
                    className="w-10 h-12 bg-slate-200 border border-white rounded flex items-center justify-center relative"
                    initial={{ scale: 0, opacity: 0 }}
                    animate={{ scale: [0, 1.2, 1], opacity: 1 }}
                    transition={{ duration: 1, repeat: Infinity, repeatDelay: 1 }}
                >
                    <div className="absolute top-0 right-0 w-3 h-3 bg-slate-300"></div>
                </motion.div>
            </div>
            <div className="absolute bottom-4 text-xs text-slate-500 font-mono">touch file.py</div>
        </div>
    );
};

const SetupEnvAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden gap-8">
            {/* ROS Core / Install */}
            <div className="flex flex-col items-center z-10">
                <div className="w-16 h-16 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-2xl">📦</span>
                </div>
                <span className="text-xs text-slate-500">ROS 2 Install</span>
            </div>

            {/* Flowing Particles */}
            <div className="flex gap-1">
                {[0, 1, 2].map(i => (
                    <motion.div
                        key={i}
                        className="w-2 h-2 bg-orange-500 rounded-full"
                        animate={{ x: [0, 100], opacity: [0, 1, 0] }}
                        transition={{ duration: 1.5, repeat: Infinity, delay: i * 0.3 }}
                    />
                ))}
            </div>

            {/* Terminal / User Env */}
            <div className="flex flex-col items-center z-10">
                <motion.div
                    className="w-16 h-16 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2"
                    animate={{ borderColor: ['#475569', '#f97316', '#475569'] }}
                    transition={{ duration: 2, repeat: Infinity }}
                >
                    <span className="text-2xl">💻</span>
                </motion.div>
                <span className="text-xs text-slate-500">Your Terminal</span>
            </div>

            <div className="absolute bottom-4 text-xs text-slate-400 font-mono">source /opt/ros/humble/setup.bash</div>
        </div>
    );
};

const NodeInspectionAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="relative">
                {/* Central Node */}
                <div className="w-24 h-24 bg-slate-800 border-2 border-orange-500 rounded-xl flex flex-col items-center justify-center z-20 relative">
                    <span className="text-slate-200 font-bold text-sm">/talker</span>
                </div>

                {/* Publishers (Outputs) */}
                <motion.div
                    className="absolute top-0 -right-24 flex items-center gap-2"
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 0.5, duration: 0.5 }}
                >
                    <div className="h-0.5 w-8 bg-orange-500"></div>
                    <div className="px-2 py-1 bg-slate-800 border border-orange-500/50 rounded text-[10px] text-orange-300">
                        Pub: /chatter
                    </div>
                </motion.div>

                {/* Subscribers (Inputs) */}
                <motion.div
                    className="absolute bottom-0 -left-24 flex items-center gap-2"
                    initial={{ opacity: 0, x: 20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 1, duration: 0.5 }}
                >
                    <div className="px-2 py-1 bg-slate-800 border border-blue-500/50 rounded text-[10px] text-blue-300">
                        Sub: /params
                    </div>
                    <div className="h-0.5 w-8 bg-blue-500"></div>
                </motion.div>
            </div>
        </div>
    );
};

const NodeRemapAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <motion.div
                className="w-32 h-24 bg-slate-800 border-2 border-slate-600 rounded-xl flex flex-col items-center justify-center z-10"
                animate={{
                    borderColor: ['#475569', '#f97316', '#475569'],
                    scale: [1, 1.1, 1]
                }}
                transition={{ duration: 3, repeat: Infinity }}
            >
                <div className="text-xs text-slate-500 mb-1">Node Name</div>
                <div className="relative h-6 overflow-hidden w-full text-center">
                    <motion.div
                        className="absolute w-full text-slate-300 font-bold"
                        animate={{ y: [0, -30, -30, 0] }}
                        transition={{ duration: 3, repeat: Infinity, times: [0, 0.4, 0.9, 1] }}
                    >
                        /talker
                    </motion.div>
                    <motion.div
                        className="absolute w-full text-orange-400 font-bold"
                        initial={{ y: 30 }}
                        animate={{ y: [30, 0, 0, 30] }}
                        transition={{ duration: 3, repeat: Infinity, times: [0, 0.4, 0.9, 1] }}
                    >
                        /my_talker
                    </motion.div>
                </div>
            </motion.div>
            <div className="absolute bottom-8 text-[10px] text-slate-500 font-mono">--remap __node:=my_talker</div>
        </div>
    );
};

const ParameterSetAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden gap-8">
            <div className="w-48 p-4 bg-slate-800 border border-slate-700 rounded-lg">
                <div className="text-xs text-slate-400 mb-3">/turtlesim Parameters</div>

                <div className="flex items-center justify-between mb-2">
                    <span className="text-xs text-slate-300">background_r</span>
                    <motion.div
                        className="px-2 py-0.5 rounded text-xs font-mono"
                        animate={{
                            backgroundColor: ['#1e293b', '#450a0a', '#1e293b'],
                            color: ['#94a3b8', '#f87171', '#94a3b8']
                        }}
                        transition={{ duration: 3, repeat: Infinity }}
                    >
                        <motion.span
                            animate={{ opacity: [1, 0, 0, 1] }}
                            transition={{ duration: 3, repeat: Infinity, times: [0, 0.4, 0.9, 1] }}
                        >69</motion.span>
                        <motion.span
                            className="absolute ml-[-14px]"
                            initial={{ opacity: 0 }}
                            animate={{ opacity: [0, 1, 1, 0] }}
                            transition={{ duration: 3, repeat: Infinity, times: [0, 0.4, 0.9, 1] }}
                        >255</motion.span>
                    </motion.div>
                </div>

                <div className="h-1 w-full bg-slate-700 rounded overflow-hidden">
                    <motion.div
                        className="h-full bg-orange-500"
                        animate={{ width: ['20%', '100%', '20%'] }}
                        transition={{ duration: 3, repeat: Infinity }}
                    />
                </div>
            </div>
        </div>
    );
};

const NodeAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="absolute inset-0 grid grid-cols-8 grid-rows-4 gap-4 opacity-10">
                {Array.from({ length: 32 }).map((_, i) => (
                    <div key={i} className="bg-slate-500 rounded-full w-1 h-1" />
                ))}
            </div>

            <motion.div
                className="w-24 h-24 bg-slate-800 border-2 border-orange-500 rounded-xl flex flex-col items-center justify-center z-10 shadow-[0_0_30px_rgba(249,115,22,0.2)]"
                animate={{
                    scale: [1, 1.05, 1],
                    borderColor: ['#f97316', '#fb923c', '#f97316']
                }}
                transition={{ duration: 3, repeat: Infinity }}
            >
                <div className="w-3 h-3 bg-green-500 rounded-full mb-2 shadow-[0_0_10px_rgba(34,197,94,0.6)] animate-pulse"></div>
                <span className="text-slate-200 font-bold text-sm">Node</span>
                <span className="text-xs text-slate-500">/talker</span>
            </motion.div>

            {/* Code lines effect */}
            <div className="absolute right-10 top-10 space-y-2 opacity-30">
                <motion.div className="h-1 w-12 bg-slate-400 rounded" animate={{ opacity: [0.3, 1, 0.3] }} transition={{ duration: 2, repeat: Infinity, delay: 0 }} />
                <motion.div className="h-1 w-20 bg-slate-400 rounded" animate={{ opacity: [0.3, 1, 0.3] }} transition={{ duration: 2, repeat: Infinity, delay: 0.5 }} />
                <motion.div className="h-1 w-16 bg-slate-400 rounded" animate={{ opacity: [0.3, 1, 0.3] }} transition={{ duration: 2, repeat: Infinity, delay: 1 }} />
            </div>
        </div>
    );
};

const TopicAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center gap-12 bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            {/* Publisher */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-xs font-bold text-slate-300">Publisher</span>
                </div>
                <span className="text-xs text-slate-500">Node A</span>
            </div>

            {/* Topic Pipe */}
            <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-48 h-12 bg-slate-800/50 rounded-full border border-slate-700 flex items-center px-2">
                <span className="absolute -top-6 left-1/2 -translate-x-1/2 text-xs text-orange-400 font-mono">/topic_name</span>
            </div>

            {/* Message Packet */}
            <motion.div
                className="w-6 h-6 bg-orange-500 rounded shadow-[0_0_15px_rgba(249,115,22,0.6)] z-20 absolute left-[calc(50%-6rem)]"
                animate={{
                    x: [0, 190],
                    opacity: [0, 1, 1, 0],
                    scale: [0.5, 1, 1, 0.5]
                }}
                transition={{
                    duration: 2,
                    repeat: Infinity,
                    ease: "easeInOut"
                }}
            />

            {/* Subscriber */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-xs font-bold text-slate-300">Subscriber</span>
                </div>
                <span className="text-xs text-slate-500">Node B</span>
            </div>
        </div>
    );
};

const ServiceAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center gap-16 bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            {/* Client */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2">
                    <span className="text-xs font-bold text-slate-300">Client</span>
                </div>
                <span className="text-xs text-slate-500">Node A</span>
            </div>

            {/* Request */}
            <motion.div
                className="absolute top-[35%] left-[calc(50%-5rem)] text-[10px] text-blue-400 font-mono"
                animate={{ x: [0, 140], opacity: [1, 0] }}
                transition={{ duration: 1.5, repeat: Infinity, repeatDelay: 1.5 }}
            >
                REQ &gt;&gt;
            </motion.div>
            <motion.div
                className="w-3 h-3 bg-blue-500 rounded-full z-20 absolute top-[42%] left-[calc(50%-5rem)]"
                animate={{ x: [0, 160], opacity: [1, 0] }}
                transition={{ duration: 1.5, repeat: Infinity, repeatDelay: 1.5 }}
            />

            {/* Response */}
            <motion.div
                className="absolute bottom-[35%] right-[calc(50%-5rem)] text-[10px] text-green-400 font-mono"
                animate={{ x: [0, -140], opacity: [0, 1] }}
                transition={{ duration: 1.5, repeat: Infinity, delay: 1.5, repeatDelay: 1.5 }}
            >
                &lt;&lt; RES
            </motion.div>
            <motion.div
                className="w-3 h-3 bg-green-500 rounded-full z-20 absolute bottom-[42%] right-[calc(50%-5rem)]"
                animate={{ x: [0, -160], opacity: [0, 1] }}
                transition={{ duration: 1.5, repeat: Infinity, delay: 1.5, repeatDelay: 1.5 }}
            />

            {/* Server */}
            <div className="flex flex-col items-center z-10">
                <div className="w-20 h-20 bg-slate-800 border border-slate-600 rounded-lg flex items-center justify-center mb-2 relative">
                    <span className="text-xs font-bold text-slate-300">Server</span>
                    <motion.div
                        className="absolute -top-2 -right-2 w-4 h-4 bg-yellow-500 rounded-full flex items-center justify-center text-[8px] text-black font-bold"
                        animate={{ scale: [0, 1, 0] }}
                        transition={{ duration: 3, repeat: Infinity, times: [0.4, 0.5, 0.9] }}
                    >
                        !
                    </motion.div>
                </div>
                <span className="text-xs text-slate-500">Node B</span>
            </div>
        </div>
    );
};

const TurtlesimHelloAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="w-64 h-48 bg-blue-500/10 flex items-center justify-center relative">
                <motion.div
                    className="text-4xl"
                    initial={{ scale: 0, rotate: -180 }}
                    animate={{ scale: 1, rotate: 0 }}
                    transition={{ type: "spring", stiffness: 260, damping: 20 }}
                >
                    🐢
                </motion.div>
                <div className="absolute top-2 left-2 text-[10px] text-blue-300">Turtlesim Window</div>
            </div>
        </div>
    );
};

const TeleopKeysAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden gap-8">
            <div className="grid grid-cols-3 gap-1">
                <div />
                <motion.div
                    className="w-8 h-8 border border-slate-600 rounded flex items-center justify-center text-slate-400"
                    animate={{ backgroundColor: ['#1e293b', '#334155', '#1e293b'] }}
                    transition={{ duration: 1, repeat: Infinity }}
                >↑</motion.div>
                <div />
                <motion.div
                    className="w-8 h-8 border border-slate-600 rounded flex items-center justify-center text-slate-400"
                    animate={{ backgroundColor: ['#1e293b', '#334155', '#1e293b'] }}
                    transition={{ duration: 1, repeat: Infinity, delay: 0.3 }}
                >←</motion.div>
                <motion.div
                    className="w-8 h-8 border border-slate-600 rounded flex items-center justify-center text-slate-400"
                    animate={{ backgroundColor: ['#1e293b', '#334155', '#1e293b'] }}
                    transition={{ duration: 1, repeat: Infinity, delay: 0.6 }}
                >↓</motion.div>
                <motion.div
                    className="w-8 h-8 border border-slate-600 rounded flex items-center justify-center text-slate-400"
                    animate={{ backgroundColor: ['#1e293b', '#334155', '#1e293b'] }}
                    transition={{ duration: 1, repeat: Infinity, delay: 0.9 }}
                >→</motion.div>
            </div>
            <div className="text-2xl">➡️</div>
            <motion.div
                className="text-4xl"
                animate={{ x: [0, 10, 0, -10, 0], y: [0, -10, 0, 10, 0] }}
                transition={{ duration: 2, repeat: Infinity }}
            >
                🐢
            </motion.div>
        </div>
    );
};

const TopicCmdVelAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden gap-4">
            <div className="w-32 h-20 bg-slate-950 border border-slate-700 rounded p-2 text-[8px] font-mono text-slate-400">
                $ ros2 topic pub...
            </div>
            <motion.div
                className="w-2 h-2 bg-orange-500 rounded-full"
                animate={{ x: [0, 60], opacity: [0, 1, 0] }}
                transition={{ duration: 1, repeat: Infinity }}
            />
            <motion.div
                className="text-4xl"
                animate={{ x: [0, 20, 0] }}
                transition={{ duration: 1, repeat: Infinity }}
            >
                🐢
            </motion.div>
        </div>
    );
};

const TopicCircleAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <motion.div
                className="text-4xl"
                animate={{ rotate: 360 }}
                transition={{ duration: 3, repeat: Infinity, ease: "linear" }}
                style={{ originX: 2, originY: 0.5 }}
            >
                🐢
            </motion.div>
        </div>
    );
};

const ServiceTeleportAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden w-full">
            <div className="relative w-64 h-32 bg-blue-500/5 rounded border border-blue-500/20">
                <motion.div
                    className="absolute text-4xl"
                    animate={{
                        opacity: [1, 0, 0, 1],
                        x: [20, 20, 180, 180],
                        y: [20, 20, 60, 60],
                        scale: [1, 0, 0, 1]
                    }}
                    transition={{ duration: 2, repeat: Infinity, times: [0, 0.4, 0.6, 1] }}
                >
                    🐢
                </motion.div>
                <div className="absolute top-2 left-2 text-[10px] text-blue-300">/teleport_absolute</div>
            </div>
        </div>
    );
};

const ServicePenAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="relative w-64 h-32 bg-blue-500/5 rounded border border-blue-500/20 flex items-center justify-center">
                <motion.div
                    className="absolute w-full h-1 bg-slate-600"
                    style={{ top: '50%' }}
                />
                <motion.div
                    className="absolute w-full h-2 bg-red-500"
                    style={{ top: '50%' }}
                    initial={{ width: 0 }}
                    animate={{ width: '100%' }}
                    transition={{ duration: 2, repeat: Infinity }}
                />
                <motion.div
                    className="text-4xl z-10 relative"
                    animate={{ x: [-100, 100] }}
                    transition={{ duration: 2, repeat: Infinity }}
                >
                    🐢
                </motion.div>
            </div>
        </div>
    );
};

const ServiceResetAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="relative w-64 h-32 bg-blue-500/5 rounded border border-blue-500/20 overflow-hidden">
                {/* Messy lines */}
                <motion.div
                    className="absolute inset-0"
                    animate={{ opacity: [1, 0, 0, 1] }}
                    transition={{ duration: 3, repeat: Infinity, times: [0, 0.1, 0.9, 1] }}
                >
                    <svg className="w-full h-full stroke-slate-500 stroke-2 fill-none">
                        <path d="M10,10 Q50,50 90,10 T150,50" />
                        <path d="M20,80 L100,20 L150,90" />
                    </svg>
                </motion.div>

                {/* Flash */}
                <motion.div
                    className="absolute inset-0 bg-white"
                    animate={{ opacity: [0, 0.5, 0] }}
                    transition={{ duration: 3, repeat: Infinity, times: [0, 0.1, 0.5] }}
                />

                <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-4xl">
                    🐢
                </div>
            </div>
        </div>
    );
};

const ServiceSpawnAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="flex gap-12">
                <div className="text-4xl opacity-50">🐢</div>
                <motion.div
                    className="text-4xl"
                    initial={{ scale: 0, opacity: 0 }}
                    animate={{ scale: 1, opacity: 1 }}
                    transition={{ duration: 0.5, repeat: Infinity, repeatDelay: 2 }}
                >
                    🐢
                </motion.div>
            </div>
        </div>
    );
};

const RvizVizAnimation = () => {
    return (
        <div className="h-48 flex items-center justify-center bg-slate-900/50 rounded-lg border border-slate-800 relative overflow-hidden">
            <div className="w-64 h-40 bg-black border border-slate-700 relative perspective-500">
                {/* Grid */}
                <div className="absolute inset-0 grid grid-cols-6 grid-rows-4 gap-4 opacity-20">
                    {Array.from({ length: 24 }).map((_, i) => (
                        <div key={i} className="border border-slate-500" />
                    ))}
                </div>

                {/* Axes */}
                <div className="absolute bottom-4 left-4">
                    <div className="w-0.5 h-8 bg-blue-500 absolute bottom-0 left-0"></div>
                    <div className="w-8 h-0.5 bg-red-500 absolute bottom-0 left-0"></div>
                    <div className="w-6 h-0.5 bg-green-500 absolute bottom-0 left-0 origin-left -rotate-45"></div>
                </div>

                {/* Robot Model */}
                <motion.div
                    className="absolute top-1/2 left-1/2 w-8 h-12 bg-slate-700 border border-orange-500"
                    animate={{ rotateY: 360 }}
                    transition={{ duration: 5, repeat: Infinity, ease: "linear" }}
                />
            </div>
        </div>
    );
};
