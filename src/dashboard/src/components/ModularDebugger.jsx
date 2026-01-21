import React, { useState, useContext, createContext } from 'react';

import {
  Terminal, Layers, Video, Activity, Settings,
  Maximize, X, AlertCircle, Cpu, Save, Layout,
  Search, ChevronRight, ChevronDown, Play, Pause
} from 'lucide-react';



// --- 1. THE CONTEXT BRAIN (Context Awareness) ---



// Holds the state of the "World" and the "UI"

const RoverContext = createContext({
  selectedNode: null, // What is the user debugging? (e.g., '/drive/front_left')
  systemMode: 'IDLE', // IDLE, AUTONOMOUS, MANUAL, ERROR
  selectNode: (_nodeId) => {},
  setSystemMode: (_mode) => {}
});



// --- 2. WIDGET REGISTRY (Modular Components) ---



// A. The Log Streamer (SRD 5.3)

const LogWidget = () => (
  <div className="flex flex-col h-full bg-zinc-950 font-mono text-xs">
    <div className="flex items-center justify-between px-2 py-1 border-b border-zinc-800 bg-zinc-900">
      <span className="text-zinc-400">/rosout</span>
      <div className="flex gap-2">
        <span className="px-1 bg-zinc-800 text-blue-400 rounded">INFO</span>
        <span className="px-1 bg-zinc-800 text-rose-400 rounded">ERR</span>
      </div>
    </div>
    <div className="flex-1 overflow-auto p-2 space-y-1">
      <div className="text-zinc-500">[10:22:01] [nav_stack]: Costmap update loop started</div>
      <div className="text-emerald-500">[10:22:02] [imu_driver]: Calibration matrix loaded</div>
      <div className="text-amber-500">[10:22:05] [vision]: Low feature count in frame 2044</div>
      <div className="text-zinc-500">[10:22:06] [telemetry]: Heartbeat sent to GCS</div>
    </div>
  </div>
);



// B. 3D Viewport (SRD 3.3)

const ViewportWidget = () => (
  <div className="relative w-full h-full bg-zinc-900 flex items-center justify-center overflow-hidden">
    <div className="absolute inset-0 bg-[linear-gradient(rgba(30,30,30,1)_1px,transparent_1px),linear-gradient(90deg,rgba(30,30,30,1)_1px,transparent_1px)] bg-[size:20px_20px] opacity-20"></div>
    {/* Mock Robot */}
    <div className="w-12 h-16 bg-blue-500/20 border border-blue-400 rounded relative flex items-center justify-center">
      <div className="w-0 h-0 border-l-[5px] border-l-transparent border-r-[5px] border-r-transparent border-b-[10px] border-b-blue-400 absolute -top-3"></div>
    </div>
    <div className="absolute top-2 right-2 bg-black/50 px-2 py-1 rounded text-xs text-zinc-400 font-mono">
      TF: map -&gt; base_link
    </div>
  </div>
);



// C. Node Graph / Tree (Context Selector)

const NodeTreeWidget = () => {
  const { selectNode, selectedNode } = useContext(RoverContext);

  const nodes = [
    { id: 'drive_system', label: 'Drive Subsystem', type: 'group', children: ['FL_Wheel', 'FR_Wheel', 'BL_Wheel', 'BR_Wheel'] },
    { id: 'arm_system', label: 'Robotic Arm', type: 'group', children: ['Shoulder', 'Elbow', 'Wrist', 'Gripper'] },
    { id: 'sensors', label: 'Sensors', type: 'group', children: ['IMU_01', 'LiDAR_Main', 'GPS_RTK'] },
  ];

  return (
    <div className="h-full overflow-auto bg-zinc-925 p-2">
      {nodes.map(group => (
        <div key={group.id} className="mb-2">
          <div className="flex items-center text-zinc-400 text-xs font-bold uppercase tracking-wider mb-1">
            <ChevronDown className="w-3 h-3 mr-1" /> {group.label}
          </div>
          <div className="pl-4 space-y-0.5">
            {group.children.map(child => (
              <div
                key={child}
                onClick={() => selectNode(child)}
                className={`cursor-pointer px-2 py-1 rounded text-sm flex items-center gap-2
                  ${selectedNode === child ? 'bg-blue-600 text-white' : 'text-zinc-400 hover:bg-zinc-800'}
                `}
              >
                <Cpu className="w-3 h-3 opacity-70" />
                {child}
              </div>
            ))}
          </div>
        </div>
      ))}
    </div>
  );
};



// --- 3. CONTEXT-AWARE INSPECTOR PANEL ---

// This is the "Smart" part. It changes entirely based on what you clicked.

const InspectorPanel = () => {
  const { selectedNode } = useContext(RoverContext);

  if (!selectedNode) return (
    <div className="h-full flex flex-col items-center justify-center text-zinc-600 p-4 text-center">
      <Search className="w-8 h-8 mb-2 opacity-50" />
      <p className="text-xs">Select a node from the tree to inspect parameters and streams.</p>
    </div>
  );

  return (
    <div className="h-full flex flex-col">
      <div className="p-3 border-b border-zinc-800 bg-zinc-900">
        <h3 className="text-sm font-bold text-white flex items-center gap-2">
          <Settings className="w-4 h-4 text-blue-400" />
          {selectedNode}
        </h3>
        <span className="text-[10px] font-mono text-zinc-500 uppercase">Node Status: ACTIVE (30Hz)</span>
      </div>

      {/* Dynamic Content based on selection */}
      <div className="p-4 space-y-4 overflow-auto flex-1">

        {/* Parameter Tuning (SRD 9.2) */}
        <div className="space-y-2">
          <label className="text-xs font-bold text-zinc-500 uppercase">PID Parameters</label>
          <div className="grid grid-cols-2 gap-2">
            <div className="bg-black p-2 rounded border border-zinc-800">
              <span className="text-xs text-zinc-500 block">P_GAIN</span>
              <span className="text-emerald-400 font-mono">0.450</span>
            </div>
            <div className="bg-black p-2 rounded border border-zinc-800">
              <span className="text-xs text-zinc-500 block">I_GAIN</span>
              <span className="text-emerald-400 font-mono">0.002</span>
            </div>
          </div>
        </div>

        {/* Live Data Plot */}
        <div className="h-32 bg-black border border-zinc-800 rounded relative">
          <span className="absolute top-1 left-2 text-[10px] text-zinc-500">OUTPUT VELOCITY</span>
          {/* Mock Sparkline */}
          <div className="absolute bottom-0 left-0 right-0 h-16 flex items-end px-1 gap-1">
            {[...Array(20)].map((_, i) => (
              <div key={i} className="w-full bg-blue-500/40" style={{ height: `${Math.random() * 100}%` }} />
            ))}
          </div>
        </div>

        <div className="pt-2">
          <button className="w-full bg-zinc-800 hover:bg-zinc-700 text-xs py-2 rounded text-zinc-300 border border-zinc-700 transition">
            Restart Node
          </button>
        </div>

      </div>
    </div>
  );
};



// --- 4. WORKSPACE CONTAINER ---



const WorkspaceTile = ({ title, children, className = "" }) => (
  <div className={`flex flex-col bg-zinc-925 border border-zinc-800 rounded-md overflow-hidden shadow-sm ${className}`}>
    <div className="h-7 flex items-center justify-between px-2 bg-zinc-900 border-b border-zinc-800 select-none">
      <div className="flex items-center gap-2">
        <Layers className="w-3 h-3 text-zinc-600" />
        <span className="text-[11px] font-bold text-zinc-400 uppercase tracking-wide">{title}</span>
      </div>
      <div className="flex gap-1">
        <Maximize className="w-3 h-3 text-zinc-600 hover:text-zinc-300 cursor-pointer" />
      </div>
    </div>
    <div className="flex-1 min-h-0 relative">
      {children}
    </div>
  </div>
);



// --- 5. MAIN APPLICATION ---



export default function ModularDebugger() {
  // State
  const [activeLayout, setActiveLayout] = useState('DEBUG'); // DEBUG, DRIVE, CALIB
  const [selectedNode, setSelectedNode] = useState(null);

  // Context Value
  const contextValue = {
    selectedNode,
    selectNode: setSelectedNode,
    systemMode: 'MANUAL',
    setSystemMode: () => {}
  };

  return (
    <RoverContext.Provider value={contextValue}>
      <div className="flex h-screen w-full bg-black text-zinc-300 font-sans overflow-hidden">

        {/* A. The "Dock" (Left Sidebar) */}
        <div className="w-12 bg-zinc-950 border-r border-zinc-800 flex flex-col items-center py-4 z-20">
          <div className="mb-6 w-8 h-8 bg-blue-600 rounded flex items-center justify-center font-bold text-white text-xs">
            OS
          </div>
          <div className="space-y-2 w-full flex flex-col items-center">
            <button onClick={() => setActiveLayout('DRIVE')} title="Mission Control" className={`p-2 rounded-lg transition ${activeLayout === 'DRIVE' ? 'bg-zinc-800 text-blue-400' : 'text-zinc-600 hover:text-zinc-400'}`}>
              <Activity className="w-5 h-5" />
            </button>
            <button onClick={() => setActiveLayout('DEBUG')} title="System Debugger" className={`p-2 rounded-lg transition ${activeLayout === 'DEBUG' ? 'bg-zinc-800 text-blue-400' : 'text-zinc-600 hover:text-zinc-400'}`}>
              <Layout className="w-5 h-5" />
            </button>
             <button onClick={() => setActiveLayout('CALIB')} title="Calibration" className={`p-2 rounded-lg transition ${activeLayout === 'CALIB' ? 'bg-zinc-800 text-blue-400' : 'text-zinc-600 hover:text-zinc-400'}`}>
              <Settings className="w-5 h-5" />
            </button>
          </div>
          <div className="mt-auto">
             <button className="p-2 text-rose-500 hover:bg-rose-950/50 rounded-lg">
               <AlertCircle className="w-5 h-5" />
             </button>
          </div>
        </div>

        {/* B. Main Workspace Area */}
        <div className="flex-1 flex flex-col h-full min-w-0">

          {/* Top Status Bar */}
          <header className="h-10 bg-zinc-950 border-b border-zinc-800 flex items-center justify-between px-4">
             <div className="flex items-center gap-4">
               <span className="text-xs font-mono text-zinc-500">PROFILE: <span className="text-zinc-200">{activeLayout}</span></span>
               <div className="h-4 w-px bg-zinc-800"></div>
               <div className="flex items-center gap-2 text-xs">
                  <span className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse"></span>
                  <span className="text-emerald-500 font-bold">ROS2 BRIDGE CONNECTED</span>
               </div>
             </div>
             <div className="flex items-center gap-2">
                <button className="flex items-center gap-1 bg-zinc-900 hover:bg-zinc-800 border border-zinc-800 px-3 py-1 rounded text-xs transition">
                  <Save className="w-3 h-3" />
                  <span>Save Layout</span>
                </button>
             </div>
          </header>

          {/* Dynamic Grid Layout Engine */}
          <main className="flex-1 p-2 overflow-hidden bg-black">

            {/* LAYOUT 1: DEBUGGING (3-Column) */}
            {activeLayout === 'DEBUG' && (
              <div className="grid grid-cols-12 grid-rows-6 gap-2 h-full">
                {/* Left: Node Hierarchy */}
                <WorkspaceTile title="System Topology" className="col-span-2 row-span-6">
                  <NodeTreeWidget />
                </WorkspaceTile>

                {/* Center: Visualization & Logs */}
                <div className="col-span-7 row-span-6 flex flex-col gap-2">
                  <WorkspaceTile title="3D Visualization (RVIZ-Web)" className="flex-[2]">
                    <ViewportWidget />
                  </WorkspaceTile>
                  <WorkspaceTile title="System Logs (/rosout)" className="flex-1">
                    <LogWidget />
                  </WorkspaceTile>
                </div>

                {/* Right: Context Aware Inspector */}
                <WorkspaceTile title="Inspector" className="col-span-3 row-span-6 border-l-4 border-l-blue-500/20">
                  <InspectorPanel />
                </WorkspaceTile>
              </div>
            )}

            {/* LAYOUT 2: DRIVE (2-Column Focus) */}
            {activeLayout === 'DRIVE' && (
              <div className="grid grid-cols-12 grid-rows-1 gap-2 h-full">
                <WorkspaceTile title="Front Navigation Cam" className="col-span-9">
                   <div className="w-full h-full bg-zinc-900 flex items-center justify-center">
                      <Video className="w-16 h-16 text-zinc-700" />
                   </div>
                </WorkspaceTile>
                <div className="col-span-3 flex flex-col gap-2">
                   <WorkspaceTile title="Local Map" className="flex-1">
                      <div className="w-full h-full bg-zinc-900"></div>
                   </WorkspaceTile>
                   <WorkspaceTile title="Telemetry" className="flex-1">
                      <div className="p-4 space-y-2">
                         <div className="flex justify-between text-sm text-zinc-400 border-b border-zinc-800 pb-1">
                           <span>VELOCITY</span>
                           <span className="text-white font-mono">1.2 m/s</span>
                         </div>
                         <div className="flex justify-between text-sm text-zinc-400 border-b border-zinc-800 pb-1">
                           <span>HEADING</span>
                           <span className="text-white font-mono">145 deg</span>
                         </div>
                      </div>
                   </WorkspaceTile>
                </div>
              </div>
            )}

          </main>
        </div>

      </div>
    </RoverContext.Provider>
  );
}
