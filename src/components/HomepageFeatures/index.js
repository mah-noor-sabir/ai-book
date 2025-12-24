import React from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

export default function Home() {
  return (
    <>
      {/* ================= HERO SECTION ================= */}
      <section className={styles.hero}>
        <div className="container">
          <div className={styles.grid}>

            {/* TEXT SIDE */}
            <div className={styles.text}>
              <span className={styles.badge}>
              🤖 A Practical Robotics Handbook
              </span>

              <Heading as="h1" className={styles.title}>
                Physical AI <br />
                <span>& Humanoid Robotics</span>
              </Heading>

              <p className={styles.description}>
                A comprehensive, hands-on guide to embodied intelligence.
                Explore how intelligent machines perceive, reason, and act
                in the physical world — from high-fidelity simulation to
                real-world robotic deployment.
              </p>

              <div className={styles.metrics}>
                <div>
                  <strong>Foundations</strong>
                  <span> AI · Control · Physics</span>
                </div>
                <div>
                  <strong>Tooling</strong>
                  <span> ROS 2 · Gazebo · Isaac Sim</span>
                </div>
                <div>
                  <strong>Practice</strong>
                  <span> Projects · Systems · Deployment</span>
                </div>
              </div>

              <div className={styles.actions}>

                <Link
                  className="button button--secondary button--lg"
                  to="/docs"
                >
                  Let's get started 
                </Link>
              </div>
            </div>

            {/* IMAGE SIDE */}
            <div className={styles.visual}>
              <div className={styles.frame}>
                <img
                  src="/img/ROB.jpeg"
                  alt="Humanoid robot research model"
                  className={styles.animatedImage}
                />
              </div>
            </div>

          </div>
        </div>
      </section>

      {/* ================= WHY PHYSICAL AI ================= */}
      <section className={styles.whySection}>
        <div className="container">

          <div className={styles.whyHeader}>
            <Heading as="h2" className={styles.whyTitle}>
              Why Physical AI?
            </Heading>
            <p className={styles.whySubtitle}>
              Physical AI integrates intelligence with physical embodiment, enabling autonomous systems to function intelligently and safely in real-world, dynamic environments.
            </p>
          </div>

          <div className={styles.whyGrid}>
            {/* LEFT VISUAL */}
            <div className={styles.whyVisual}>
              <div className={styles.whyCard}>
                <img
                  src="/img/ROBO.jpeg"
                  alt="Physical AI illustration"
                  className={styles.whyIcon}
                />
              </div>
            </div>

            {/* RIGHT CONTENT */}
            <div className={styles.whyContent}>
              <div className={styles.whyItem}>
                <h4>Real-Time Perception & Decision-Making</h4>
                <p>
                  Learn how embodied agents fuse sensor data, reason under
                  uncertainty, and generate control policies in real time.
                </p>
              </div>

              <div className={styles.whyItem}>
                <h4>End-to-End Robotics Pipelines</h4>
                <p>
                  Full implementations using ROS 2, Gazebo, and NVIDIA Isaac Sim,
                  bridging simulation and physical hardware.
                </p>
              </div>

              <div className={styles.whyItem}>
                <h4>From Fundamentals to Advanced VLA Models</h4>
                <p>
                  Structured coverage of perception, control, and
                  vision-language-action models powering modern humanoids.
                </p>
              </div>

              <div className={styles.whyItem}>
                <h4>Integrated AI Assistance</h4>
                <p>
                  A built-in AI chatbot provides instant explanations,
                  implementation guidance, and conceptual clarity throughout
                  the learning journey.
                </p>
              </div>
            </div>
          </div>

        </div>
      </section>
    </>
  );
}
