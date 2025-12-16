import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import useBaseUrl from '@docusaurus/useBaseUrl'; // Import useBaseUrl

const FeatureList = [
  {
    title: 'The Robotic Nervous System (ROS 2)',
    icon: '🤖', // Reverted to emoji
    description: 'Master the middleware for robot control. Learn about ROS 2 Nodes, Topics, and Services and how to bridge Python agents to ROS controllers using rclpy.',
    link: 'docs/module-1-ros/what-is-ros' // Cleaned link
  },
  {
    title: 'The Digital Twin (Gazebo & Unity)',
    icon: '🌍', // Reverted to emoji
    description: 'Explore physics simulation and environment building. Simulate physics, gravity, and collisions in Gazebo, and create high-fidelity human-robot interactions in Unity.',
    link: 'docs/module-2-digital-twin/intro-to-simulation' // Cleaned link
  },
  {
    title: 'The AI-Robot Brain (NVIDIA Isaac™)',
    icon: '🚀', // Reverted to emoji
    description: 'Dive into advanced perception and training. Use NVIDIA Isaac Sim for photorealistic simulation and synthetic data generation, and Isaac ROS for hardware-accelerated VSLAM and navigation.',
    link: 'docs/module-3-nvidia-isaac/intro-to-nvidia-isaac' // Cleaned link
  },
];

function Feature({ title, icon, description, link, isReversed }) {
  const finalLink = useBaseUrl(link); // Use the hook to get the full URL
  const content = (
    <div className={clsx('col col--6', styles.featureContent)}>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        <a href={finalLink}>Learn more...</a>
    </div>
  );

  const iconContainer = (
    <div className={clsx('col col--6', styles.featureIconContainer)}>
        <div className={styles.featureIcon}>{icon}</div> {/* Reverted to render emoji as text */}
    </div>
  );

  return (
    <div className={clsx('row', styles.featureRow, isReversed && styles.featureRowReversed)}>
      {isReversed ? content : iconContainer}
      {isReversed ? iconContainer : content}
    </div>
  );
}


export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <Heading as="h2" className={clsx('text--center', styles.sectionHeading)}>Key Concepts</Heading>
        {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} isReversed={idx % 2 !== 0} />
        ))}
      </div>
    </section>
  );
}