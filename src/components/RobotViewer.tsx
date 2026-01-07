import React, { useRef, useState, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Stage, PerspectiveCamera, Html, useGLTF } from '@react-three/drei';
import * as THREE from 'three';

function RobotBody({ activePart, onPartClick }: { activePart: string | null, onPartClick: (name: string) => void }) {
  const meshRef = useRef<THREE.Group>(null);

  // Create a procedural robot model using primitives
  return (
    <group ref={meshRef}>
      {/* Head */}
      <mesh position={[0, 1.6, 0]} onClick={(e) => { e.stopPropagation(); onPartClick('Head (AI Processing)'); }}>
        <boxGeometry args={[0.4, 0.4, 0.3]} />
        <meshStandardMaterial color={activePart === 'Head (AI Processing)' ? '#3b82f6' : '#222'} emissive={activePart === 'Head (AI Processing)' ? '#3b82f6' : '#000'} />
      </mesh>

      {/* Neck */}
      <mesh position={[0, 1.4, 0]}>
        <cylinderGeometry args={[0.08, 0.1, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>

      {/* Torso */}
      <mesh position={[0, 1.0, 0]} onClick={(e) => { e.stopPropagation(); onPartClick('Core (Nervous System)'); }}>
        <boxGeometry args={[0.6, 0.8, 0.4]} />
        <meshStandardMaterial color={activePart === 'Core (Nervous System)' ? '#3b82f6' : '#333'} />
      </mesh>

      {/* Arms */}
      <group position={[0.4, 1.3, 0]}>
        <mesh position={[0, -0.3, 0]} rotation={[0, 0, 0.1]}>
          <boxGeometry args={[0.15, 0.6, 0.15]} />
          <meshStandardMaterial color="#444" />
        </mesh>
      </group>
      <group position={[-0.4, 1.3, 0]}>
        <mesh position={[0, -0.3, 0]} rotation={[0, 0, -0.1]}>
          <boxGeometry args={[0.15, 0.6, 0.15]} />
          <meshStandardMaterial color="#444" />
        </mesh>
      </group>

      {/* Legs */}
      <mesh position={[0.2, 0.3, 0]}>
        <boxGeometry args={[0.2, 0.6, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>
      <mesh position={[-0.2, 0.3, 0]}>
        <boxGeometry args={[0.2, 0.6, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>

      {/* Ground shadows */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]} receiveShadow>
        <planeGeometry args={[10, 10]} />
        <meshStandardMaterial transparent opacity={0.1} color="black" />
      </mesh>
    </group>
  );
}

const RobotViewer: React.FC = () => {
  const [activePart, setActivePart] = useState<string | null>(null);

  const getSpecs = (part: string) => {
    switch(part) {
      case 'Head (AI Processing)': return "Blackwell B200 Dual-SoC • 4D Perception • YOLOv11 Engine";
      case 'Core (Nervous System)': return "Jetson Thor Integration • ROS 2 Kilted Kaiju Core • 500 TOPS AI Performance";
      default: return "Select a component to view 2026 specifications";
    }
  };

  return (
    <div style={{ width: '100%', height: '700px', backgroundColor: '#0a0a0f', borderRadius: '24px', position: 'relative', overflow: 'hidden', border: '1px solid #27272a' }}>
      <Canvas shadows dpr={[1, 2]}>
        <PerspectiveCamera makeDefault position={[0, 1.5, 4]} fov={50} />
        <ambientLight intensity={0.5} />
        <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} intensity={1} castShadow />
        <pointLight position={[-10, -10, -10]} intensity={0.5} />

        <RobotBody activePart={activePart} onPartClick={setActivePart} />

        <OrbitControls enableZoom={true} enablePan={false} minPolarAngle={Math.PI/4} maxPolarAngle={Math.PI/2} />
      </Canvas>

      <div style={{ position: 'absolute', bottom: '40px', left: '40px', right: '40px', padding: '24px', background: 'rgba(13, 13, 18, 0.8)', backdropFilter: 'blur(20px)', borderRadius: '16px', border: '1px solid rgba(59, 130, 246, 0.2)', pointerEvents: 'none' }}>
        <h4 style={{ color: '#3b82f6', marginBottom: '8px', fontSize: '1.2rem' }}>{activePart || "Interactive 2026 Humanoid Viewer"}</h4>
        <p style={{ color: '#a1a1aa', margin: 0 }}>{getSpecs(activePart || "")}</p>
      </div>

      <div style={{ position: 'absolute', top: '40px', right: '40px' }}>
        <div style={{ padding: '8px 16px', background: 'rgba(59, 130, 246, 0.1)', color: '#3b82f6', borderRadius: '99px', fontSize: '0.8rem', fontWeight: 'bold' }}>
          3D INTERACTIVE MODE
        </div>
      </div>
    </div>
  );
};

export default RobotViewer;
