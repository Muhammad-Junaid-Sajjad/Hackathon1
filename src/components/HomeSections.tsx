// Homepage layout sections
export function HeroSection({ styles, Heading, Link, clsx, StatItem, InteractiveHero }: any) {
  return (
    <header className={styles.hero}>
      <div className={styles.heroBackground}>
        <div className={styles.gridLines} />
        <div className={styles.glowOrb1} /><div className={styles.glowOrb2} /><div className={styles.glowOrb3} />
      </div>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.heroBadge}><span className={styles.badgeDot} />Interactive 2026 Robotics Textbook</div>
          <Heading as="h1" className={clsx(styles.heroTitle, styles.neonTitle)}>
             <span className={styles.titleLine}>Physical AI &</span>
             <span className={clsx(styles.titleLine, styles.gradientText)}>Humanoid Robotics</span>
          </Heading>
          <p className={styles.heroSubtitle}>Final 2026 Frontier Architecture: Master ROS 2 Kilted Kaiju, NVIDIA Isaac Sim, and VLA Foundation Models.</p>
          <div className={styles.heroCta}>
            <Link className={clsx('button', 'button--lg', styles.primaryButton)} to="/docs/M1/C1/m1-c1-s1">Get Started</Link>
          </div>
          <div className={styles.heroStats}>
            <StatItem number="87" label="Sections" delay={200} />
            <StatItem number="1,500+" label="Projects" delay={400} />
            <StatItem number="10k+" label="Students" delay={600} />
          </div>
        </div>
        <div className={styles.heroVisual}><InteractiveHero /></div>
      </div>
    </header>
  );
}
