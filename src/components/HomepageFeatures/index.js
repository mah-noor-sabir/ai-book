import React from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

export default function HomepageHero() {
  return (
    <section className={styles.hero}>
      <div className="container">
        <div className={styles.grid}>
          {/* TEXT SIDE */}
          <div className={styles.text}>
            <span className={styles.badge}>
              🤖 A Practical Robotics Handbook
            </span>

            <Heading as="h1" className={styles.title}>
              Physical AI  <br />
              <span>& Humanoid Robotics</span>
            </Heading>

            <p className={styles.description}>
              A hands-on journey into humanoid robotics and embodied AI.
              Learn how intelligent machines perceive, decide, and act
              in the real world — from simulation to physical deployment.
            </p>

            <div className={styles.metrics}>
              <div>
                <strong>Foundations</strong>
                <span>AI · Control · Physics</span>
              </div>
              <div>
                <strong>Tooling</strong>
                <span>ROS 2 · Gazebo · Isaac</span>
              </div>
              <div>
                <strong>Practice</strong>
                <span>Projects · Systems</span>
              </div>
            </div>

            <div className={styles.actions}>
              <Link className={`${styles.primaryBtn} button button--secondary button--lg`} to="/docs/intro">
                Explore the Book
              </Link>

              <Link className={`${styles.primaryBtn} button button--secondary button--lg`} to="/docs/intro">
                View Chapters
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
  );
}
