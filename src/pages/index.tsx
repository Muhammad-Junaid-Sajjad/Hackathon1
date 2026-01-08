import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import VscodeInterface from '@site/src/components/VscodeInterface';
import RobotViewer from '@site/src/components/RobotViewer';

// Simple Stat Item Component
function StatItem({ number, label }: { number: string; label: string }) {
  return (
    <div className={styles.statItem}>
      <span className={styles.statNumber}>{number}</span>
      <span className={styles.statLabel}>{label}</span>
    </div>
  );
}

// Hero Section
function Hero() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.gridLines} />
      </div>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}>2026 Frontier Edition</div>
          <Heading as="h1" className={clsx(styles.heroTitle, styles.gradientText)}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>
            {siteConfig.tagline}
          </p>
          <div className={styles.heroCta}>
            <Link className={clsx('button button--lg button--primary', styles.primaryButton)} to="/docs/M1/C1/m1-c1-s1">
              Start Learning →
            </Link>
            <Link className={clsx('button button--lg button--secondary', styles.secondaryButton)} to="/docs/intro">
              Explore Curriculum
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

// Modules Grid
function ModulesGrid() {
  const modules = [
    { n: 'M1', t: 'Nervous System', d: 'ROS 2 Foundations', c: '#6366f1' },
    { n: 'M2', t: 'Digital Twin', d: 'Physics & Gazebo', c: '#8b5cf6' },
    { n: 'M3', t: 'AI Awakening', d: 'Isaac Sim & RL', c: '#a855f7' },
    { n: 'M4', t: 'Embodiment', d: 'Vision-Language-Action', c: '#d946ef' },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className="text--center padding-horiz--md">Complete Curriculum</h2>
        <div className={styles.modulesGrid}>
          {modules.map((m, i) => (
            <Link to={`/docs/M${i+1}/C1/m${i+1}-c1-s1`} key={i} className={styles.moduleCard}>
              <div className={styles.moduleNumber} style={{ background: m.c }}>{m.n}</div>
              <div className={styles.moduleContent}>
                <h3 className={styles.moduleTitle}>{m.t}</h3>
                <p className={styles.moduleDescription}>{m.d}</p>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

// Stats Section
function StatsDashboard() {
  return (
    <section className={styles.statsSection}>
      <div className="container padding-vert--lg">
        <div className={styles.statsGrid}>
          <StatItem number="10,000+" label="Students Learning" />
          <StatItem number="87+" label="Sections Completed" />
          <StatItem number="1,500+" label="Projects Built" />
          <StatItem number="45+" label="Countries" />
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Home`}
      description="Physical AI & Humanoid Robotics Textbook - 2026 Edition">
      <Hero />

      <main>
        <StatsDashboard />

        <section className="container padding-vert--xl">
          <div className="row">
            <div className="col col--6">
              <h2>🤖 Interactive 3D Robot Viewer</h2>
              <p>Explore the humanoid robot anatomy with our interactive 3D viewer. Understand the nervous system architecture and component relationships.</p>
            </div>
            <div className="col col--6">
              <RobotViewer />
            </div>
          </div>
        </section>

        <ModulesGrid />

        <section className="container padding-vert--xl">
          <div className="row">
            <div className="col">
              <h2 className="text--center">💻 Live Development Environment</h2>
              <p className="text--center padding-horiz--md">Experience hands-on learning with our integrated VS Code simulator. Write, test, and debug robotics code in real-time.</p>
              <div style={{ height: '500px', marginTop: '2rem' }}>
                <VscodeInterface />
              </div>
            </div>
          </div>
        </section>

        <section className="padding-vert--xl text--center" style={{ background: 'var(--ifm-color-primary-dark)' }}>
          <div className="container">
            <h2 className={styles.gradientText}>Ready to Build the Future?</h2>
            <p style={{ color: 'rgba(255,255,255,0.8)', maxWidth: '600px', margin: '0 auto 2rem' }}>
              Join thousands of students mastering the next generation of robotics and AI.
            </p>
            <Link className="button button--lg button--secondary" to="/docs/M1/C1/m1-c1-s1">
              Start Learning Today
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
