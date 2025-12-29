/**
 * Homepage - Physical AI & Humanoid Robotics Textbook
 *
 * Design inspired by: Figure.ai (futuristic robotics), GitHub (clean), Panaversity (educational)
 */

import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Animated Robot SVG Component
function RobotIllustration() {
  return (
    <div className={styles.robotContainer}>
      <svg
        viewBox="0 0 400 400"
        className={styles.robotSvg}
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Glowing background circle */}
        <circle cx="200" cy="200" r="180" fill="url(#glow)" className={styles.glowCircle} />

        {/* Robot head */}
        <rect x="130" y="80" width="140" height="120" rx="20" fill="url(#headGradient)" className={styles.robotHead} />

        {/* Eyes */}
        <circle cx="165" cy="130" r="18" fill="#0a0a0f" />
        <circle cx="235" cy="130" r="18" fill="#0a0a0f" />
        <circle cx="165" cy="130" r="10" fill="#818cf8" className={styles.eyeGlow} />
        <circle cx="235" cy="130" r="10" fill="#818cf8" className={styles.eyeGlow} />

        {/* Antenna */}
        <line x1="200" y1="80" x2="200" y2="50" stroke="url(#antennaGradient)" strokeWidth="4" />
        <circle cx="200" cy="45" r="8" fill="#818cf8" className={styles.antennaLight} />

        {/* Body */}
        <rect x="120" y="210" width="160" height="100" rx="15" fill="url(#bodyGradient)" />

        {/* Chest display */}
        <rect x="150" y="230" width="100" height="50" rx="8" fill="#0a0a0f" />
        <rect x="160" y="240" width="80" height="4" rx="2" fill="#818cf8" className={styles.displayLine} />
        <rect x="160" y="250" width="60" height="4" rx="2" fill="#6366f1" className={styles.displayLine} />
        <rect x="160" y="260" width="70" height="4" rx="2" fill="#818cf8" className={styles.displayLine} />

        {/* Arms */}
        <rect x="70" y="220" width="40" height="80" rx="10" fill="url(#armGradient)" className={styles.leftArm} />
        <rect x="290" y="220" width="40" height="80" rx="10" fill="url(#armGradient)" className={styles.rightArm} />

        {/* Legs */}
        <rect x="140" y="320" width="35" height="60" rx="8" fill="url(#legGradient)" />
        <rect x="225" y="320" width="35" height="60" rx="8" fill="url(#legGradient)" />

        {/* Gradients */}
        <defs>
          <radialGradient id="glow" cx="0.5" cy="0.5" r="0.5">
            <stop offset="0%" stopColor="#818cf8" stopOpacity="0.3" />
            <stop offset="100%" stopColor="#818cf8" stopOpacity="0" />
          </radialGradient>
          <linearGradient id="headGradient" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stopColor="#4f46e5" />
            <stop offset="100%" stopColor="#7c3aed" />
          </linearGradient>
          <linearGradient id="bodyGradient" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stopColor="#3730a3" />
            <stop offset="100%" stopColor="#5b21b6" />
          </linearGradient>
          <linearGradient id="armGradient" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#4338ca" />
            <stop offset="100%" stopColor="#6366f1" />
          </linearGradient>
          <linearGradient id="legGradient" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#3730a3" />
            <stop offset="100%" stopColor="#4f46e5" />
          </linearGradient>
          <linearGradient id="antennaGradient" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#818cf8" />
            <stop offset="100%" stopColor="#6366f1" />
          </linearGradient>
        </defs>
      </svg>
    </div>
  );
}

// Stats counter component
function StatItem({ number, label, delay }: { number: string; label: string; delay: number }) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setIsVisible(true), delay);
    return () => clearTimeout(timer);
  }, [delay]);

  return (
    <div className={clsx(styles.statItem, isVisible && styles.visible)}>
      <span className={styles.statNumber}>{number}</span>
      <span className={styles.statLabel}>{label}</span>
    </div>
  );
}

// Feature card component
function FeatureCard({
  icon,
  title,
  description,
  delay,
}: {
  icon: string;
  title: string;
  description: string;
  delay: number;
}) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setIsVisible(true), delay);
    return () => clearTimeout(timer);
  }, [delay]);

  return (
    <div className={clsx(styles.featureCard, isVisible && styles.visible)}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

// Module card component
function ModuleCard({
  number,
  title,
  description,
  chapters,
  link,
  color,
}: {
  number: string;
  title: string;
  description: string;
  chapters: number;
  link: string;
  color: string;
}) {
  return (
    <Link to={link} className={styles.moduleCard}>
      <div className={styles.moduleNumber} style={{ background: color }}>
        {number}
      </div>
      <div className={styles.moduleContent}>
        <h3 className={styles.moduleTitle}>{title}</h3>
        <p className={styles.moduleDescription}>{description}</p>
        <div className={styles.moduleChapters}>
          <span>📚 {chapters} Chapters</span>
          <span className={styles.moduleArrow}>→</span>
        </div>
      </div>
    </Link>
  );
}

// Hero section
function HeroSection() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.hero}>
      {/* Animated background */}
      <div className={styles.heroBackground}>
        <div className={styles.gridLines} />
        <div className={styles.glowOrb1} />
        <div className={styles.glowOrb2} />
        <div className={styles.glowOrb3} />
      </div>

      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          {/* Badge */}
          <div className={styles.heroBadge}>
            <span className={styles.badgeDot} />
            Open Source Textbook • Free Forever
          </div>

          {/* Title */}
          <Heading as="h1" className={styles.heroTitle}>
            <span className={styles.titleLine}>Physical AI &</span>
            <span className={clsx(styles.titleLine, styles.gradientText)}>
              Humanoid Robotics
            </span>
          </Heading>

          {/* Subtitle */}
          <p className={styles.heroSubtitle}>
            The definitive guide to building intelligent physical systems.
            Master ROS 2, simulation, perception, and deploy production-ready robots.
          </p>

          {/* CTA Buttons */}
          <div className={styles.heroCta}>
            <Link
              className={clsx('button', 'button--lg', styles.primaryButton)}
              to="/docs/M1/C1/m1-c1-s1"
            >
              Start Learning
              <span className={styles.buttonIcon}>→</span>
            </Link>
            <Link
              className={clsx('button', 'button--lg', styles.secondaryButton)}
              to="/docs/glossary"
            >
              Explore Content
            </Link>
          </div>

          {/* Stats */}
          <div className={styles.heroStats}>
            <StatItem number="84+" label="Sections" delay={200} />
            <StatItem number="4" label="Modules" delay={400} />
            <StatItem number="200+" label="Terms" delay={600} />
            <StatItem number="∞" label="Knowledge" delay={800} />
          </div>
        </div>

        {/* Robot Illustration */}
        <div className={styles.heroVisual}>
          <RobotIllustration />
        </div>
      </div>

      {/* Scroll indicator */}
      <div className={styles.scrollIndicator}>
        <span>Scroll to explore</span>
        <div className={styles.scrollLine} />
      </div>
    </header>
  );
}

// Features section
function FeaturesSection() {
  const features = [
    {
      icon: '🤖',
      title: 'ROS 2 Mastery',
      description: 'Learn the industry-standard Robot Operating System from fundamentals to advanced patterns.',
    },
    {
      icon: '🌐',
      title: 'Digital Twins',
      description: 'Build realistic simulations with Gazebo and NVIDIA Isaac for safe development.',
    },
    {
      icon: '👁️',
      title: 'AI Perception',
      description: 'Implement computer vision, LiDAR processing, and sensor fusion for robot intelligence.',
    },
    {
      icon: '🦾',
      title: 'Humanoid Control',
      description: 'Master locomotion, manipulation, and whole-body control for humanoid robots.',
    },
    {
      icon: '🧠',
      title: 'VLA Integration',
      description: 'Connect Vision-Language-Action models for next-gen autonomous capabilities.',
    },
    {
      icon: '🚀',
      title: 'Production Ready',
      description: 'Deploy robust systems with industry-proven patterns and safety considerations.',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className={styles.sectionContainer}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Features</span>
          <h2 className={styles.sectionTitle}>
            Everything you need to build
            <span className={styles.gradientText}> intelligent robots</span>
          </h2>
          <p className={styles.sectionSubtitle}>
            A comprehensive curriculum designed for all skill levels,
            from curious beginners to seasoned robotics engineers.
          </p>
        </div>

        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <FeatureCard
              key={index}
              icon={feature.icon}
              title={feature.title}
              description={feature.description}
              delay={index * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

// Modules section
function ModulesSection() {
  const modules = [
    {
      number: 'M1',
      title: 'ROS 2 Foundations',
      description: 'Core concepts, nodes, topics, services, and development environment setup.',
      chapters: 3,
      link: '/docs/M1/C1/m1-c1-s1',
      color: 'linear-gradient(135deg, #6366f1, #8b5cf6)',
    },
    {
      number: 'M2',
      title: 'Simulation & Digital Twins',
      description: 'Gazebo simulation, sensor modeling, and sim-to-real transfer.',
      chapters: 3,
      link: '/docs/M2/C1/m2-c1-s1',
      color: 'linear-gradient(135deg, #8b5cf6, #a855f7)',
    },
    {
      number: 'M3',
      title: 'Perception & AI',
      description: 'Computer vision, deep learning, NVIDIA Isaac, and world models.',
      chapters: 3,
      link: '/docs/M3/C1/m3-c1-s1',
      color: 'linear-gradient(135deg, #a855f7, #d946ef)',
    },
    {
      number: 'M4',
      title: 'Integration & Deployment',
      description: 'VLA models, cloud robotics, fleet management, and production systems.',
      chapters: 3,
      link: '/docs/M4/C1/m4-c1-s1',
      color: 'linear-gradient(135deg, #d946ef, #ec4899)',
    },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className={styles.sectionContainer}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Curriculum</span>
          <h2 className={styles.sectionTitle}>
            Four modules.
            <span className={styles.gradientText}> Complete mastery.</span>
          </h2>
          <p className={styles.sectionSubtitle}>
            Progress through structured learning paths that take you from
            beginner concepts to production-ready implementations.
          </p>
        </div>

        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <ModuleCard key={index} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}

// AI Features section
function AIFeaturesSection() {
  return (
    <section className={styles.aiSection}>
      <div className={styles.sectionContainer}>
        <div className={styles.aiContent}>
          <div className={styles.aiText}>
            <span className={styles.sectionBadge}>AI-Powered</span>
            <h2 className={styles.sectionTitle}>
              Learn smarter with
              <span className={styles.gradientText}> AI assistance</span>
            </h2>
            <p className={styles.sectionSubtitle}>
              Our intelligent features adapt to your learning style and help you
              master robotics faster than ever before.
            </p>

            <div className={styles.aiFeaturesList}>
              <div className={styles.aiFeature}>
                <div className={styles.aiFeatureIcon}>💬</div>
                <div>
                  <h4>RAG Chatbot</h4>
                  <p>Ask questions and get instant answers from the textbook content.</p>
                </div>
              </div>
              <div className={styles.aiFeature}>
                <div className={styles.aiFeatureIcon}>✨</div>
                <div>
                  <h4>Personalization</h4>
                  <p>Content adapts to your background, goals, and preferred depth.</p>
                </div>
              </div>
              <div className={styles.aiFeature}>
                <div className={styles.aiFeatureIcon}>🌐</div>
                <div>
                  <h4>Urdu Translation</h4>
                  <p>Access content in Urdu for better understanding.</p>
                </div>
              </div>
            </div>

            <Link
              className={clsx('button', 'button--lg', styles.primaryButton)}
              to="/auth/signup"
            >
              Create Free Account
            </Link>
          </div>

          <div className={styles.aiVisual}>
            <div className={styles.chatPreview}>
              <div className={styles.chatHeader}>
                <span className={styles.chatDot} />
                <span className={styles.chatDot} />
                <span className={styles.chatDot} />
                <span>AI Assistant</span>
              </div>
              <div className={styles.chatMessages}>
                <div className={styles.chatUser}>
                  How do I create a ROS 2 node?
                </div>
                <div className={styles.chatBot}>
                  <p>To create a ROS 2 node, follow these steps:</p>
                  <code>ros2 pkg create my_package</code>
                  <p>Then implement your node class extending Node...</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// CTA section
function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.sectionContainer}>
        <div className={styles.ctaContent}>
          <h2 className={styles.ctaTitle}>
            Ready to build the
            <span className={styles.gradientText}> future of robotics?</span>
          </h2>
          <p className={styles.ctaSubtitle}>
            Join thousands of engineers, researchers, and enthusiasts learning
            Physical AI and Humanoid Robotics.
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button', 'button--lg', styles.primaryButton)}
              to="/docs/M1/C1/m1-c1-s1"
            >
              Start Learning Now
              <span className={styles.buttonIcon}>→</span>
            </Link>
            <Link
              className={clsx('button', 'button--lg', styles.secondaryButton)}
              to="https://github.com/junaid/Hackathon-1"
            >
              ⭐ Star on GitHub
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

// Main Home component
export default function Home(): React.ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="The definitive guide to building intelligent physical systems. Master ROS 2, simulation, perception, and deploy production-ready robots."
    >
      <HeroSection />
      <FeaturesSection />
      <ModulesSection />
      <AIFeaturesSection />
      <CTASection />
    </Layout>
  );
}
